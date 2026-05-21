import argparse
import numpy as np
import torch
import matplotlib.pyplot as plt
import cv2
import os
print("CWD:", os.getcwd())
from pathlib import Path
import sys

# Import a function from the pytorch-superpoint submodule
curr_path = os.path.dirname(os.path.abspath(__file__))
os.chdir(curr_path)
sys.path.append("../thirdparty/pytorch-superpoint")
from utils.loader import get_module

def load_model(opt):
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")

    # Construct the model config
    model_dict = {'name': 'SuperPointNet_gauss2',
                  'params': {},
                  'detection_threshold': opt.detection_threshold,
                  'nms': opt.nms_dist,
                  'nn_thresh': 1.0,
                  'pretrained': opt.superpoint_model_path,
                  'batch_size': 1}

    # Load the model
    model_module = get_module("", 'Val_model_heatmap')
    model = model_module(model_dict, device=device)
    model.loadModel()

    return model, device


def load_kdsp_model(opt):
    repo_root = Path(curr_path).resolve().parents[3]
    sys.path.append(str(repo_root))
    from thermal_superpoint.models.student import ThermalSuperPoint

    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    model = ThermalSuperPoint(
        nms_radius=opt.nms_dist,
        detection_threshold=opt.detection_threshold,
        descriptor_dim=256,
    ).to(device)

    ckpt = torch.load(opt.kdsp_ckpt, map_location="cpu")
    state = ckpt.get("model", ckpt)
    model.load_state_dict(state)
    model.eval()
    return model, device

def read_image(width, height, path):
    input_image = cv2.imread(path)
    if input_image is None:
        raise FileNotFoundError(f"Failed to read image: {path}")
    if not width is None:
        input_image = cv2.resize(input_image, (width, height), interpolation=cv2.INTER_AREA)
    input_image = cv2.cvtColor(input_image, cv2.COLOR_RGB2GRAY)
    input_image_float = input_image.astype('float32') / 255.0
    H, W = input_image_float.shape[0], input_image_float.shape[1]
    return input_image, torch.tensor(input_image_float, dtype=torch.float32).reshape(1, 1, H, W)

def get_superpoint_features(model, device, img):
    model.run(img.to(device))

    # heatmap to pts
    pts = model.heatmap_to_pts()
    
    # subpixel estimation
    pts = model.soft_argmax_points(pts, patch_size=5)

    # heatmap, pts to desc
    desc_sparse = model.desc_to_sparseDesc()

    return np.asarray(pts[0], dtype=np.float32).T, np.asarray(desc_sparse[0], dtype=np.float32).T


def get_kdsp_features(model, device, img):
    with torch.no_grad():
        out = model({"thermal": img.to(device)})
    kpts = out["keypoints"][0].detach().cpu().numpy()
    scores = out["keypoint_scores"][0].detach().cpu().numpy()
    desc = out["descriptors"][0].detach().cpu().numpy()
    if kpts.shape[0] == 0:
        return np.zeros((0, 3), dtype=np.float32), np.zeros((0, desc.shape[1] if desc.ndim == 2 else 256), dtype=np.float32)
    kpts = np.hstack([kpts.astype(np.float32), scores.reshape(-1, 1).astype(np.float32)])
    return kpts, desc.astype(np.float32)

if __name__ == "__main__":
    # Handle arguments
    parser = argparse.ArgumentParser(description ='Applies a trained SuperPoint network to an image directory and '
        'outputs the resulting keypoints and descriptors in sequentially named YAML files.')
    parser.add_argument('superpoint_model_path', nargs='?', default=None,
        help = 'Filepath to the trained superpoint model file.')
    parser.add_argument('directory_path', type = str, help='Path to image directory')
    parser.add_argument('out_dir', type=str, 
        help='Output directory name (it will located in the same folder as the original image directory).')
    parser.add_argument('--max-features', type=int, default=100e3, 
        help='The maximum number of features to keep per image (default: 100e3).')
    parser.add_argument('--resize', type=int, default=None, nargs=2,
        help='The width and height to resize the image to (default: None, the original size is kept)')
    parser.add_argument('--detection-threshold', type=float, default=0.015, 
        help='Superpoint heatmap interest point detection threshold (default: 0.015)')
    parser.add_argument('--nms-dist', type=int, default=4, 
        help='SuperPoint Non Maximum Suppression (NMS) distance (default: 4).')
    parser.add_argument('--output-orb', action='store_true',
        help='Output ORB features instead of SuperPoint features. If True superpoint_model_path and the max-features '
        'are ignored (default: False)')
    parser.add_argument('--kdsp-ckpt', type=str, default=None,
        help='ThermalKDSuperPoint checkpoint. If set, uses KDSP instead of pytorch-superpoint.')
    opt = parser.parse_args()

    # Load the model
    if not opt.output_orb:
        if opt.kdsp_ckpt:
            print("Loading ThermalKDSuperPoint model...\n")
            model, device = load_kdsp_model(opt)
            print("Model loaded.\n")
        else:
            if not opt.superpoint_model_path:
                parser.error("superpoint_model_path is required unless --kdsp-ckpt is provided")
            print("Loading SuperPoint model...\n")
            model, device = load_model(opt)
            print("Model loaded.\n")

    # Create the ORB detector
    if opt.output_orb:
        orb = cv2.ORB_create()

    # Find paths to image file paths
    image_files = sorted([f for f in os.listdir(opt.directory_path) if f.lower().endswith(('.png', '.jpg', '.jpeg', '.bmp', '.tif', '.tiff'))])
    print('Found ' + str(len(image_files)) + ' files in ' + opt.directory_path + '\n')

    # Create output directory
    results_dir = str(Path(opt.directory_path).resolve().parent / opt.out_dir)
    if not os.path.exists(results_dir):
        os.makedirs(results_dir)
    print('Using output directory: ' + results_dir + '\n')

    # Loop over images, generate keypoints and descriptors, and log them
    for index, image_file in enumerate(sorted(image_files)):
        image_path = os.path.join(opt.directory_path, image_file)

        # Import the image
        if opt.resize is None:
            img_np, img = read_image(None, None, image_path)
        else:
            img_np, img = read_image(opt.resize[0], opt.resize[1], image_path)
    
        # Generate keypoints and descriptors
        if not opt.output_orb:
            if opt.kdsp_ckpt:
                kpts, desc = get_kdsp_features(model, device, img)
            else:
                kpts, desc = get_superpoint_features(model, device, img)
        else:
            kpts, desc = orb.detectAndCompute(img_np, None)
            kpts = np.asarray([[kp.pt[0], kp.pt[1], kp.response] for kp in kpts])

        # Keep only the top max points (300 in the original bag of binary words paper)
        if not opt.output_orb and opt.max_features < kpts.shape[0]:
            pts = np.hstack((kpts, desc))
            pts = pts[np.argsort(pts[:, 2])]
            kpts = pts[-opt.max_features:, :3]
            desc = pts[-opt.max_features:, 3:]
    
        # Write the results to a yaml file
        result_file = cv2.FileStorage(results_dir + '/' + str(index + 1) + '.yaml', 1)
        result_file.write('keypoints', kpts)
        result_file.write('descriptors', desc)
        result_file.release()

import cv2
import numpy as np
import torch
from torchvision import transforms
from segmentation_models_pytorch import DeepLabV3Plus
from PIL import Image
import os

# Set device
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

# Define the model loading function
def load_model(model_path, device, num_classes=1):
    model = DeepLabV3Plus(
        encoder_name="efficientnet-b0",
        encoder_weights=None,
        in_channels=3,
        classes=num_classes
    ).to(device)
    model.load_state_dict(torch.load(model_path, map_location=device))
    model.eval()
    return model


# Load Potholes and Lane Edges Models
# (Don't preload cracks due to tiling)
MODELS = {
    "potholes": load_model(os.path.join(os.path.dirname(__file__), "../../models/potholes_efficientnet_b0.pth")),
    "lane_edges": load_model(os.path.join(os.path.dirname(__file__), "../../models/lanes_efficientnet_b0.pth")),
}


# Preprocessing (for potholes/lane_edges)
def preprocess_image(cv_image):
    preprocess = transforms.Compose([
        transforms.Resize((512, 512)),
        transforms.ToTensor(),
        transforms.Normalize(mean=[0.5, 0.5, 0.5], std=[0.5, 0.5, 0.5])
    ])
    pil_image = Image.fromarray(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))
    return preprocess(pil_image).unsqueeze(0).to(device)

# General Inference Function
def segment_defect(cv_image, defect_type, **kwargs):
    if defect_type == "cracks":
        return segment_cracks(cv_image, **kwargs)

    model = MODELS.get(defect_type)
    if model is None:
        raise ValueError(f"No model loaded for defect type: {defect_type}")
    
    input_tensor = preprocess_image(cv_image)
    with torch.no_grad():
        output = model(input_tensor)
        output = torch.sigmoid(output).squeeze().cpu().numpy()
        binary_mask = (output > 0.5).astype(np.uint8) * 255

    return cv2.resize(binary_mask, (cv_image.shape[1], cv_image.shape[0]), interpolation=cv2.INTER_NEAREST)

def preprocess_tile(pil_image):
    preprocess = transforms.Compose([
        transforms.Resize((448, 448)),
        transforms.ToTensor(),
        transforms.Normalize(mean=[0.5, 0.5, 0.5], std=[0.5, 0.5, 0.5])
    ])
    return preprocess(pil_image)

def segment_tile(tile, model, device):
    pil_tile = Image.fromarray(cv2.cvtColor(tile, cv2.COLOR_BGR2RGB))
    input_tensor = preprocess_tile(pil_tile).unsqueeze(0).to(device)

    with torch.no_grad():
        output = model(input_tensor)
        output = torch.sigmoid(output).squeeze().cpu().numpy()
        mask = (output > 0.5).astype(np.uint8) * 255
    return cv2.resize(mask, (tile.shape[1], tile.shape[0]), interpolation=cv2.INTER_NEAREST)


# Crack Inference (Tiling)
def segment_cracks(cv_image, tile_size=448, overlap=0):
    h, w, _ = cv_image.shape
    model_path = os.path.join(os.path.dirname(__file__), "../../models/cracks_efficientnet_b0.pth")
    model = load_model(model_path)

    full_mask = np.zeros((h, w), dtype=np.uint8)

    for y in range(0, h, tile_size - overlap):
        for x in range(0, w, tile_size - overlap):
            tile = cv_image[y:y+tile_size, x:x+tile_size]
            th, tw, _ = tile.shape

            if th < tile_size or tw < tile_size:
                padded_tile = np.zeros((tile_size, tile_size, 3), dtype=np.uint8)
                padded_tile[:th, :tw] = tile
                mask_tile = segment_tile(padded_tile, model)[:th, :tw]
            else:
                mask_tile = segment_tile(tile, model)

            full_mask[y:y+th, x:x+tw] = np.maximum(full_mask[y:y+th, x:x+tw], mask_tile)

    return full_mask
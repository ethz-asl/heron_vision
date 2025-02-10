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
def load_model(model_path, num_classes=1):
    model = DeepLabV3Plus(
        encoder_name="efficientnet-b0",
        encoder_weights=None,
        in_channels=3,
        classes=num_classes
    ).to(device)
    model.load_state_dict(torch.load(model_path, map_location=device))
    model.eval()
    return model

# Load models once (avoid reloading on every function call)
MODELS = {
    "pothole": load_model(os.path.join(os.path.dirname(__file__), "../../models/potholes_efficientnet_b0.pth")),
    "crack": load_model(os.path.join(os.path.dirname(__file__), "../../models/cracks_efficientnet_b0.pth")),
    "lane_edge": load_model(os.path.join(os.path.dirname(__file__), "../../models/lanes_efficientnet_b0.pth")),
}

# Preprocessing function
def preprocess_image(cv_image):
    preprocess = transforms.Compose([
        transforms.Resize((256, 256)),
        transforms.ToTensor(),
        transforms.Normalize(mean=[0.5, 0.5, 0.5], std=[0.5, 0.5, 0.5])
    ])
    pil_image = Image.fromarray(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))
    return preprocess(pil_image).unsqueeze(0).to(device)  # Add batch dimension

# Inference function
def segment_defect(cv_image, defect_type, **kwargs):
    """
    Runs inference for the specified defect type.

    Args:
        cv_image (np.ndarray): Input image (BGR format).
        defect_type (str): "pothole", "crack", or "lane_edge"
        **kwargs: Additional parameters (e.g., is_right_side for laneEdge)

    Returns:
        np.ndarray: Segmentation mask (0 and 255)
    """
    if defect_type not in MODELS:
        raise ValueError(f"Invalid defect type: {defect_type}")

    print("defect is ",defect_type)
    model = MODELS[defect_type]
    input_tensor = preprocess_image(cv_image)

    with torch.no_grad():
        output = model(input_tensor)
        output = torch.sigmoid(output).squeeze(0).squeeze(0).cpu().numpy()

    # Convert output to binary mask
    binary_mask = (output > 0.5).astype(np.uint8) * 255  

    return cv2.resize(binary_mask, (cv_image.shape[1], cv_image.shape[0]), interpolation=cv2.INTER_NEAREST)

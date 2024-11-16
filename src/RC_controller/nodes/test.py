#!/usr/bin/env python3

import os
from tensorflow.keras.models import load_model

# Get the absolute path to the model file
SCRIPT_PATH = os.path.dirname(os.path.realpath(__file__)) + "/"
model_path = SCRIPT_PATH + "testmodel3.keras"

# Try loading the model
try:
    model = load_model(model_path)
    print("Model loaded successfully.")
except Exception as e:
    print(f"Error loading model: {e}")

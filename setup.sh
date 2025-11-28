#!/bin/bash

# Initialize and update git submodules
echo "Initializing submodules..."
git submodule update --init --recursive

echo "Setup complete!"

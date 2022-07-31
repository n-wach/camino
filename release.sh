#!/bin/sh

python -m build
read -p "Camino PyPI token: " token
TWINE_USERNAME=__token__ TWINE_PASSWORD=$token twine upload dist/*

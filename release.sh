#!/bin/sh

echo "Remember to tag and push the current commit!"
# for instance:
# git tag -a 1.0.4 -m "Release 1.0.4"
# git push origin --tags

python -m build
read -p "Camino PyPI token: " token
TWINE_USERNAME=__token__ TWINE_PASSWORD=$token twine upload dist/*

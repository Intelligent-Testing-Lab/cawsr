#!/bin/bash
# clone into current directory
git clone https://github.com/Intelligent-Testing-Lab/cawsr_workspace.git
find . -maxdepth 1 ! -name ".git" ! -name ".gitignore" ! -name "requirements.txt" ! -name "README.md" ! -name "docker-compose.yml" -exec mv {} ../ \;

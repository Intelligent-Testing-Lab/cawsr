# Development
## Setup

Firstly, ensure you have the dependencies installed, and successfuly set up CAWSR following the [README.md](../README.md).
Next, move `dev/Dockerfile` to the root directory and build the development image:
```bash
docker build -f Dockerfile -t cawsr_dev .
```

This will create a docker image tagged `cawsr_dev` that you can use for development.

## Workspace
To pull the cawsr workspace into the root of the repository, use the following script:
```bash
chmod +x dev/pull_workspace.sh
./dev/pull_workspace.sh
```
This will be mounted into the dev container to allow you to configure CAWSR as usual. See [README.md](../README.md) for more information on configuring CAWSR.

## Running CAWSR
The `dev/docker-compose.yml` file includes the services needed to launch the development container. Ensure the file is in the root of the repository and run:
```bash
docker compose -f dev/docker-compose.yml up cawsr autoware-latest
```

Development follows a similar pattern to the operation of CAWSR - simply start the autoware container and CAWSR. The `cawsr_dev` image mounts root of the repository into the container, so any changes you make are applied in CAWSR. If you need to configure CAWSR, either edit `dev/.env` to change the runtime configuration, or `configs/` to configure CAWSR.

## Contributing
See [CONTRIBUTING.md](../CONTRIBUTING.md)

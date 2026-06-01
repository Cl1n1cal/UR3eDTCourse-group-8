# UR3e Digital Twin

## Steps to run the DT

### Clone the repository
To clone the repository and enter the project directory, run:
```bash
git clone https://github.com/Cl1n1cal/UR3eDTCourse-group-8.git
cd UR3eDTCourse-group-8
```
### Install Dependencies

This project is managed using [Poetry](https://python-poetry.org/). To install the dependencies, run the following in the project root, preferably in a virtual environment:

```bash
poetry install
```
### Execution Permissions

The visualization executables need permission to run, the following commands will enable it:
```bash
chmod +x ur3e_dt_visualization/exports/linux/UR3e.x86_64
chmod +x ur3e_dt_visualization/exports/windows/UR3e.exe
```

### Activate virtual environment

To start the DT, the virtual environment needs to be active, run:
```bash
source .venv/bin/activate
```

### Start all services

You are now ready to start the DT with:
```bash
python3 -m startup.start_all_services
```
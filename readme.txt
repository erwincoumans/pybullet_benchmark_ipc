A rudimentary re-creation of some of the fixtures, adapted to work with Bullet (using PyBullet Python bindings)
Related to https://ipc-sim.github.io/rigid-ipc

Usage on Linux, Windows or MacOS:

pip3 install pybullet

For videos, make sure ffmpeg is installed and in the path.


python benchmark.py --create_video=<1 or 0> --json_file=<path> --shift_mag=<-1,0,1>
--create_video=1 requires ffmpeg working (and in the path)
--json_file=<relative_path> is relative to current working directory
--shift_mag=<-1,0,1>, Wavefront OBJ files are not centered around origin, some benchmarks require a positive (1), negative (-1) or no shift (0)
--default_friction=1
--enable_sat=1, use separating axis test (SAT) for collision

For example:

python benchmark.py --json_file=fixtures/3D/friction/incline-plane/slopeTest_highSchoolPhysics_mu=0.5.json --enable_sat=1
python benchmark.py --json_file=fixtures/3D/chain/anchor.json --shift_mag=1 --default_friction=0.5
python benchmark.py --json_file=fixtures/3D/friction/card-house/card-house-3-levels.json --create_video=1
python benchmark.py --json_file fixtures/3D/unit-tests/cube-falling-on-edge.json --create_video=1 --shift_mag=1
python benchmark.py --json_file=fixtures/3D/friction/arch/arch-101-stones.json --shift_mag=1 --create_video=0 --camera_distance=70 --camera_target_y=20
python benchmark.py --json_file=fixtures/3D/friction/arch/arch-25-stones.json --shift_mag=1 --create_video=0 --camera_distance=70 --camera_target_y=20
python benchmark.py --json_file=fixtures/3D/friction/rolling/coin.json  --camera_distance=1 --camera_target_x=0 --camera_target_z=-0.6 --camera_target_y=0.1
python benchmark.py --json_file=fixtures/3D/friction/rolling/cone.json --shift_mag=-1 --create_video=1
python benchmark.py --json_file=fixtures/3D/unit-tests/tet-corner.json --shift_mag=1 --enable_sat=1 --default_friction=0.5 --create_video=1 

PyBullet requires a mass parameter, not density.
python wrecking_ball.py --create_video=<1 or 0>

See the videos directory for videos created using those scripts.
PyBullet GUI: use mouse, in combination with CTRL and/or ALT. Press 'w' to toggle wireframe, 'g' to toggle gui

Check the PyBullet Quickstart Guide for more details about the Python bindings (link at https://pybullet.org)

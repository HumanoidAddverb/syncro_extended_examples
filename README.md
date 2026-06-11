## Syncro Extended Examples

**This repository provides additional examples for the cobot.**

Contains example for:

* **Pick And Place** : Run the robot in loop for pick and place task

Further more examples will be added to this repository

## Setup

1. Clone the repository

    ```bash
    git clone https://github.com/HumanoidAddverb/syncro_extended_examples
    ```

2. Copy the folder to the cobot PC's home directory

    ```bash
    scp -r /path/to/syncro_extended_examples cobot@192.168.0.12:~/
    ```

3. Copy the cobot shell script to the cobot PC

    The shell script is located inside the `syncro_extended_examples` repository.

    ```bash
    scp /path/to/syncro_extended_examples/cobot cobot@192.168.0.12:~/
    ```

4. Make the shell script executable

    SSH into the cobot PC and run:

    ```bash
    sudo chmod +x cobot
    ```

## Running the Example Scripts

1. Create a new container using the shell script

    ```bash
    ./cobot create
    ```

2. Navigate to the `syncro_extended_examples` folder inside the container and build

    ```bash
    cd /syncro_extended_examples
    mkdir build
    cd build
    cmake ..
    make -j
    ```

3. Run the examples

    - **Pick and place task:** see [pick_and_place/README.md](pick_and_place/README.md)

    ```bash
    ./execute_pick_place_spline pick_place_waypoints.csv
    ```

4. To run the build in test examples

    navigate to tests folder in the docker container
    ```bash
    cd /tests/build
    ```
    here you can find the heal_server executable and other examples

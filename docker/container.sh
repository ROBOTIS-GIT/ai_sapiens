#!/bin/bash

# Get the directory where the script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
CONTAINER_NAME="ai_sapiens"

# Pull the --sim flag out of the arguments; what remains is the command.
SIM=false
ARGS=()
for arg in "$@"; do
    case "$arg" in
        "--sim") SIM=true ;;
        *) ARGS+=("$arg") ;;
    esac
done
set -- "${ARGS[@]}"

COMPOSE_FILES=(-f "${SCRIPT_DIR}/docker-compose.yml")
if [ "$SIM" = true ]; then
    COMPOSE_FILES+=(-f "${SCRIPT_DIR}/docker-compose.sim.yml")
fi

# Function to display help
show_help() {
    echo "Usage: $0 [command] [--sim]"
    echo ""
    echo "Commands:"
    echo "  help                    Show this help message"
    echo "  start                   Start the container"
    echo "  enter                   Enter the running container"
    echo "  stop                    Stop the container"
    echo ""
    echo "Options:"
    echo "  --sim                   Build and run the MuJoCo simulation image"
    echo "                          locally instead of the published robot image."
    echo "                          Pass it to both start and stop."
    echo ""
    echo "Examples:"
    echo "  $0 start                Start container"
    echo "  $0 start --sim          Start container for MuJoCo sim2sim"
    echo "  $0 enter                Enter the running container"
    echo "  $0 stop                 Stop the container"
    echo "  $0 stop --sim           Stop the simulation container"
}

# Function to start the container
start_container() {
    # Set up X11 forwarding only if DISPLAY is set
    if [ -n "$DISPLAY" ]; then
        echo "Setting up X11 forwarding..."
        xhost +local:docker || true
    else
        echo "Warning: DISPLAY environment variable is not set. X11 forwarding will not be available."
    fi

    if [ "$SIM" = true ]; then
        echo "Starting ${CONTAINER_NAME} container (MuJoCo simulation)..."
    else
        echo "Starting ${CONTAINER_NAME} container..."
    fi

    ## Pull the latest images
    # docker compose "${COMPOSE_FILES[@]}" pull

    # Run docker-compose
    docker compose "${COMPOSE_FILES[@]}" up -d --build
}

# Function to enter the container
enter_container() {
    # Set up X11 forwarding only if DISPLAY is set
    if [ -n "$DISPLAY" ]; then
        echo "Setting up X11 forwarding..."
        xhost +local:docker || true
    else
        echo "Warning: DISPLAY environment variable is not set. X11 forwarding will not be available."
    fi

    if ! docker ps | grep -q "$CONTAINER_NAME"; then
        echo "Error: Container is not running"
        exit 1
    fi
    docker exec -it "$CONTAINER_NAME" bash
}

# Function to stop the container
stop_container() {
    if ! docker ps | grep -q "$CONTAINER_NAME"; then
        echo "Error: Container is not running"
        exit 1
    fi

    echo "Warning: This will stop and remove the container. All unsaved data in the container will be lost."
    read -p "Are you sure you want to continue? [y/N] " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        docker compose "${COMPOSE_FILES[@]}" down
    else
        echo "Operation cancelled."
        exit 0
    fi
}

# Main command handling
case "$1" in
    "help"|"-h"|"--help"|"")
        show_help
        ;;
    "start")
        start_container
        ;;
    "enter")
        enter_container
        ;;
    "stop")
        stop_container
        ;;
    *)
        echo "Error: Unknown command: $1"
        show_help
        exit 1
        ;;
esac

# Smart_Mob_midterm_12200276
Fleet Management System

## Overview

The Dynamic Fleet Management System is a ROS 2 (Robot Operating System 2) project that aims to distribute and route vehicles in a smart mobility service in an effective manner. For interactivity, this project requires the development of an Action Server, an Action Client, and a Command Line Interface (CLI). The system allows customers to request car allocation and routing via a simple CLI.

## Functionality

### Action Server

The Action Server is responsible for managing action requests related to vehicle allocation and routing. Its functionality includes:

- Allocating vehicles based on user requests.
- Implementing advanced allocation and routing logic.
- Simulating real-time traffic data.
- Managing vehicle constraints, e.g., battery constraints.

### Action Client

The Action Client interacts with the Action Server to send requests for vehicle allocation and routing. It provides a user-friendly interface for users to request services. The Action Client:

- Sends user requests to the Action Server.
- Processes responses from the Action Server.
- Displays allocation and routing information to the user.

### Command Line Interface (CLI)

The Command Line Interface (CLI) serves as the user interface for the Dynamic Fleet Management System. Users can interact with the system using commands and options provided by the CLI. The CLI includes commands for:

- Allocating vehicles.
- Routing vehicles.
- Listing available vehicles.
- Handling error conditions gracefully.
- Providing user feedback.

## How to Run the Project

To run the Dynamic Fleet Management System, follow these steps:

1. **Prerequisites:**
   - Ensure you have ROS 2 installed and set up on your system.
   - Create a ROS 2 workspace if you haven't already.

2. **Create a ROS 2 Package:**
   - Create a new ROS 2 package for the project, following the ROS 2 package structure.

3. **Custom Messages (If Required):**
   - If you need custom messages for your actions, define them in ROS 2 message format (`.msg` files) and place them in the `msg` directory of your package.

4. **Action Server and Action Client:**
   - Develop the Action Server and Action Client components for vehicle allocation and routing, ensuring they are integrated into your ROS 2 package.

5. **CLI Implementation:**
   - Create a Python script for your Command Line Interface (CLI) using a CLI library (e.g., `argparse`, `Click`, or `docopt`).
   - Implement CLI commands for vehicle allocation, routing, and other interactions. Handle user input, send requests to the Action Server, and display responses to the user.

6. **Testing:**
   - Test the components individually and ensure they work as expected.
   - Perform integrated testing to validate the entire system's functionality.

7. **User Documentation:**
   - Create a user manual that explains how to use the CLI and provides examples of interacting with the fleet management system.

8. **Error Handling:**
   - Implement error handling in the CLI and other components to manage issues gracefully.

9. **Integration and Building:**
   - Integrate the CLI, Action Server, and Action Client into your ROS 2 package.
   - Build your ROS 2 package using `colcon`.

10. **Running the Project:**
    - Run the fleet management system, including the Action Server, Action Client, and CLI, by executing the appropriate commands.

11. **Maintenance and Enhancements:**
    - Continue to maintain and enhance the system based on feedback and evolving requirements.


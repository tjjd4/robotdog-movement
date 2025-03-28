from model.CameraController import CameraController

if __name__ == '__main__':
    controller = CameraController()
    controller.start_server()

    print("\nCamera Controller Interactive Console")
    print("-----------------------------------")
    print("Available commands:")
    print("1. start_camera - Start the camera")
    print("2. stop_camera - Stop the camera")
    print("3. start_detection - Start object detection")
    print("4. stop_detection - Stop object detection")
    print("5. exit - Exit the application")

    print("\n\nYou can view the camera feed at http://localhost:5000/video")

    while True:
        command = input("\nEnter command: ").strip()

        if command == '1':
            controller.start_camera()
            print("Camera started.")

        elif command == '2':
            controller.stop_camera()
            print("Camera stopped.")

        elif command == '3':
            controller.start_detection()
            print("Detection started.")

        elif command == '4':
            controller.stop_detection()
            print("Detection stopped.")

        elif command == 'h':
            print("Available commands:")
            print("1. start_camera - Start the camera")
            print("2. stop_camera - Stop the camera")
            print("3. start_detection - Start object detection")
            print("4. stop_detection - Stop object detection")
            print("q. exit - Exit the application")

        elif command == 'q':
            controller.stop_camera()
            print("Exiting the application.")
            break

        else:
            print("Unknown command. Please try again. or enter 'h' to get help")
            print("\nYou can view the camera feed at http://localhost:5000/video")
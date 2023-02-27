from camera.camera import Camera

def main():
    camera = Camera()

    while True:
        print("(1) Record\n(2) Close\n")
        inp = input("Command: ")

        if inp == "1":
            while True:
                camera.record()
        
        if inp == "2":
            camera.close()
            break

if __name__ == "__main__":
    main()

import cv2
import numpy as np

def get_two_points(map_img):
    points = []
    img = map_img.copy()

    def click_event(event, x, y, flags, param):
        nonlocal points, img
        if event == cv2.EVENT_LBUTTONDOWN and len(points) < 2:
            points.append((x, y))
            cv2.circle(img, (x, y), 5, (255, 255, 255), -1)
            cv2.imshow("map", img)

            if len(points) == 2:
                cv2.destroyAllWindows()

    cv2.imshow("map", img)
    cv2.setMouseCallback("map", click_event)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    return points

# Example usage
map_img = np.zeros((800, 800, 3), dtype=np.uint8)
p1, p2 = get_two_points(map_img)
print("p1 =", p1)
print("p2 =", p2)
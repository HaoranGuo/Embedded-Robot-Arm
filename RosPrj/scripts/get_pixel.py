import cv2

img = cv2.imread('../img/hust.png')
cnt = 0
width = img.shape[1]
height = img.shape[0]

# 鼠标点击后，记录坐标到列表，最终存储在txt文件中
def get_pixel(event, x, y, flags, param):
    global cnt, width, height
    if event == cv2.EVENT_LBUTTONDOWN:
        # 显示在图片上，标明顺序
        print(x, y)
        cnt += 1
        cnt_str = str(cnt)
        cv2.circle(img, (x, y), 2, (0, 255, 0))
        cv2.putText(img, cnt_str, (x, y), cv2.FONT_HERSHEY_PLAIN,1.0, (0,255,0))
        cv2.imshow("image", img)
        with open('../data/hust.txt', 'a') as f:
            f.write(str(x/width) + ' ' + str(y/height) + ' 0\n')
    
    if event == cv2.EVENT_MBUTTONDOWN:
        print(x, y)
        cnt += 1
        cnt_str = str(cnt)
        cv2.circle(img, (x, y), 2, (0, 0, 255))
        cv2.putText(img, cnt_str, (x, y), cv2.FONT_HERSHEY_PLAIN,1.0, (0,0,255))
        cv2.imshow("image", img)
        with open('../data/hust.txt', 'a') as f:
            f.write(str(x/width) + ' ' + str(y/height) + ' 1\n')


        

cv2.namedWindow("image")
cv2.setMouseCallback("image", get_pixel)
while(1):
    cv2.imshow("image", img)
    key = cv2.waitKey(5) & 0xFF
    if key == ord('q'):
        break
cv2.destroyAllWindows()

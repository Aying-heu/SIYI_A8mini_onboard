import cv2
import numpy as np
import os


def get_perfect_four_corners(mask):
    # 1. 找轮廓
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours: return None
    max_cnt = max(contours, key=cv2.contourArea)

    # 2. 【核心】不要用 approxPolyDP，直接用最小外接矩形拟合
    # 这个函数会计算包含所有点的最小面积矩形
    # 它会顺着两条长边的趋势，在缺口处自动交叉，找回那个“虚空”的顶点
    rect = cv2.minAreaRect(max_cnt)
    
    # 3. 将 RotatedRect 转换为 4 个角点的坐标
    box = cv2.boxPoints(rect)
    box = np.array(box, dtype="float32")

    # 4. 排序：左上、右上、右下、左下
    # (这一步对 solvePnP 至关重要)
    ordered_box = order_points(box)

    # for i, pt in enumerate(ordered_box):
    #     dist = cv2.pointPolygonTest(max_cnt, (float(pt[0]), float(pt[1])), True)
    #     if dist < -5:
    #         print(f"角点 {i} 是被啃掉的方位，坐标: {pt}")
    
    return ordered_box


def get_quad_corners(mask):
    # 1. 找轮廓
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours: return None
    max_cnt = max(contours, key=cv2.contourArea)

    # 1. 提取凸包
    hull = cv2.convexHull(max_cnt)

    # 2. 多边形逼近 (在近距离下，这里通常会得到 5 个点)
    epsilon = 0.02 * cv2.arcLength(hull, True)
    approx = cv2.approxPolyDP(hull, epsilon, True)
    approx = approx.reshape(-1, 2)

    if len(approx) == 4:
        return order_points(approx.astype("float32"))
    
    if len(approx) == 5:
        # 找到“被啃掉”产生的那条短边，并剔除它，通过交点还原
        return fix_5_to_4(approx)
    
    # 如果以上都失败，回退到 minAreaRect (保底)
    rect = cv2.minAreaRect(max_cnt)
    return order_points(cv2.boxPoints(rect))

def order_points(pts):
    rect = np.zeros((4, 2), dtype="float32")
    s = pts.sum(axis=1)
    rect[0] = pts[np.argmin(s)]   # 左上：x+y 最小
    rect[2] = pts[np.argmax(s)]   # 右下：x+y 最大
    diff = np.diff(pts, axis=1)
    rect[1] = pts[np.argmin(diff)] # 右上：y-x 最小 (或 x-y 最大)
    rect[3] = pts[np.argmax(diff)] # 左下：y-x 最大
    return rect

def fix_5_to_4(pts):
    """
    针对5个点的情况，找到由于缺口产生的短边，通过两条长边延长线交点还原第4个角
    """
    n = len(pts)
    dists = [np.linalg.norm(pts[i] - pts[(i+1)%n]) for i in range(n)]
    
    # 缺口产生的边通常是凸包里最短的（或者是两个最短的之一）
    short_edge_idx = np.argmin(dists)
    
    # 找到短边相邻的两条长边
    # 边 i-1 到 i 和 边 i+1 到 i+2
    p1 = pts[(short_edge_idx - 1) % n]
    p2 = pts[short_edge_idx]
    p3 = pts[(short_edge_idx + 1) % n]
    p4 = pts[(short_edge_idx + 2) % n]
    
    # 求直线 (p1,p2) 和 (p3,p4) 的交点
    def intersect(p1, p2, p3, p4):
        x1, y1 = p1; x2, y2 = p2
        x3, y3 = p3; x4, y4 = p4
        denom = (y4-y3)*(x2-x1) - (x4-x3)*(y2-y1)
        if denom == 0: return p2 # 平行则返回原点
        ua = ((x4-x3)*(y1-y3) - (y4-y3)*(x1-x3)) / denom
        return np.array([x1 + ua*(x2-x1), y1 + ua*(y2-y1)])

    new_corner = intersect(p1, p2, p3, p4)
    
    # 构造新的 4 个点
    final_pts = []
    for i in range(n):
        if i == short_edge_idx:
            final_pts.append(new_corner)
        elif i == (short_edge_idx + 1) % n:
            continue
        else:
            final_pts.append(pts[i])
            
    return order_points(np.array(final_pts, dtype="float32"))

def extract_red_dynamic(image_path):
    if isinstance(image_path, str):
        img = cv2.imread(image_path)
        if img is None:
            print(f"错误：无法读取路径 {image_path} 下的图片！")
            return None
    # 2. 判断是否是numpy数组（直接传入的图像）
    elif isinstance(image_path, np.ndarray):
        # 检查是否是有效的图像数组（2D灰度/3D彩色）
        if len(image_path.shape) not in (2, 3):
            print("错误：输入的numpy数组不是有效的图像格式！")
            return None
        img = image_path.copy()  # 复制避免修改原数组
    # 3. 非法输入类型
    else:
        print(f"错误：输入类型 {type(image_path)} 不支持！仅支持字符串（路径）或numpy数组（图像）")
        return None

    # 1. 分离通道 (OpenCV 读进来是 BGR)
    b, g, r = cv2.split(img)

    # 2. 方法一：R通道绝对优势法 (针对你观察到的饱和现象)
    # 我们要求 R 必须比 G 高出一定程度，且比 B 高出一定程度
    # 这样可以抵消掉环境光带来的数值整体上涨
    diff_rg = cv2.subtract(r, g)
    diff_rb = cv2.subtract(r, b)
    
    # 这里的 50 是阈值，可以根据你观察到的 (255-127=128) 来调
    _, mask_rg = cv2.threshold(diff_rg, 60, 255, cv2.THRESH_BINARY)
    _, mask_rb = cv2.threshold(diff_rb, 40, 255, cv2.THRESH_BINARY)
    
    red_mask = cv2.bitwise_and(mask_rg, mask_rb)

    # 3. 方法二：YCrCb 空间 (非常推荐)
    ycrcb = cv2.cvtColor(img, cv2.COLOR_BGR2YCrCb)
    y, cr, cb = cv2.split(ycrcb)
    # Cr 代表红色差异，红色的 Cr 值通常 > 150 (默认 128 是中性)
    _, mask_cr = cv2.threshold(cr, 155, 255, cv2.THRESH_BINARY)

    # 4. 后处理：依然使用闭运算填补内部 ABC 和 ArUco
    # 形态学闭运算。先膨胀在腐蚀
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    # red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)
    # mask_cr = cv2.morphologyEx(mask_cr, cv2.MORPH_CLOSE, kernel)

    # 第一步：闭运算（先膨胀后腐蚀）- 填充孔洞
    red_mask_close = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)
    mask_cr_close = cv2.morphologyEx(mask_cr, cv2.MORPH_CLOSE, kernel)

    # 第二步：开运算（先腐蚀后膨胀）- 去除噪点/分离粘连
    red_mask_final = cv2.morphologyEx(red_mask_close, cv2.MORPH_OPEN, kernel)
    mask_cr_final = cv2.morphologyEx(mask_cr_close, cv2.MORPH_OPEN, kernel)

    # 替换原有赋值（后续用final版掩码）
    red_mask = red_mask_final
    mask_cr = mask_cr_final

    # 寻找外轮廓
    # red_mask=fill_and_fix_bitten_corner(red_mask)
    # mask_cr=fill_and_fix_bitten_corner(mask_cr)

    rect1=np.zeros((4, 2), dtype="float32")
    rect2=np.zeros((4, 2), dtype="float32")
    rect1=get_perfect_four_corners(red_mask)
    rect2=get_perfect_four_corners(mask_cr)

    if rect1 is not None and rect2 is not None:
        points = np.int32(rect1)
        points = points.reshape((-1, 1, 2))
        cv2.polylines(img, [points], isClosed=True, color=(255, 0, 0), thickness=2)
        for i, (x, y) in enumerate(rect1):
            x_int = int(x)
            y_int = int(y)
            cv2.circle(img, (x_int, y_int), 5, (255, 0, 0), -1)
            cv2.putText(img, str(i+1), (x_int+10, y_int), cv2.FONT_HERSHEY_SIMPLEX, 
                        0.8, (255, 0, 0), 2)
        points = np.int32(rect2)
        points = points.reshape((-1, 1, 2))
        cv2.polylines(img, [points], isClosed=True, color=(0, 255, 0), thickness=2)
        for i, (x, y) in enumerate(rect2):
            x_int = int(x)
            y_int = int(y)
            cv2.circle(img, (x_int, y_int), 5, (0, 255, 0), -1)
            cv2.putText(img, str(i+1), (x_int+10, y_int), cv2.FONT_HERSHEY_SIMPLEX, 
                        0.8, (0, 255, 0), 2)


    # 5. 显示对比
    cv2.imshow("Original", cv2.resize(img, (960, 540)))
    cv2.imshow("Red Mask (Diff)", cv2.resize(red_mask, (960, 540)))
    cv2.imshow("Cr Mask (YCrCb)", cv2.resize(mask_cr, (960, 540)))
    key = cv2.waitKey(1)
    return key




# folder_path="/home/bsa/A_vision_relate/STAR/data_20260117_133504/images/detected/"
# img_extensions = ['.jpg', '.jpeg', '.png', '.bmp', '.tif']
# img_list = []
# # 遍历文件夹
# for file in sorted(os.listdir(folder_path)):  # sorted保证按文件名排序
#     file_ext = os.path.splitext(file)[1].lower()
#     if file_ext in img_extensions:
#         # 拼接完整路径
#         img_path = os.path.join(folder_path, file)
#         img_list.append(img_path)
# idx=0
# total_imgs = len(img_list)
# print(f"共找到 {total_imgs} 张图片")
# while True:
#     # 边界保护：防止索引越界
#     idx = max(0, min(idx, total_imgs - 1))
#     # 读取当前索引的图片
#     current_img_path = img_list[idx]
#     frame = cv2.imread(current_img_path)
#     if frame is None:
#         print(f"读取图片失败：{current_img_path}")
#         idx += 1  # 跳过错误图片
#         continue
    
#     # 调用你的红色提取函数（传入图像数组，而非路径）
#     key = extract_red_dynamic(frame)& 0xFF  # 你的函数已兼容数组输入
    
#     # 【核心3】按键监听（关键！必须在imshow之后调用waitKey）
#     # 显示当前图片信息（可选）
#     cv2.putText(frame, f"Img: {idx+1}/{total_imgs} | {os.path.basename(current_img_path)}", 
#                 (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
#     cv2.imshow("Image Viewer", cv2.resize(frame, (960, 540)))
    
#     if key == ord('q'):  # 按q退出
#         print("退出程序...")
#         break
#     elif key == ord('d'):  # 按d下一张
#         idx += 1
#         print(f"切换到第 {idx+1} 张")
#     elif key == ord('a'):  # 按a上一张
#         idx -= 1
#         print(f"切换到第 {idx+1} 张")

# # 释放窗口
# cv2.destroyAllWindows()



cap = cv2.VideoCapture(0)
while(True):
    ret, frame = cap.read()
    if not ret:
        print("无法获取视频帧，退出...")
        break
    extract_red_dynamic(frame)

# 释放资源
# cap.release()
# 测试你的图片
# extract_red_dynamic("/home/bsa/A_vision_relate/data_20260121_200751/images/raw/1768997274738916928.jpg")
# extract_red_dynamic("/home/bsa/A_vision_relate/STAR/data_20260117_133504/images/raw/1768628273159488064.jpg")
# # # # # import os
# # # # # import cv2
# # # # # import numpy as np
# # # # # from pathlib import Path
# # # # # from typing import Dict, List, Tuple

# # # # # def get_timestamp_from_filename(filename: str) -> int:
# # # # #     """从文件名中提取纳秒级时间戳（例如：1711234567890123456.jpg -> 1711234567890123456）"""
# # # # #     try:
# # # # #         return int(os.path.splitext(filename)[0])
# # # # #     except ValueError:
# # # # #         return 0

# # # # # def get_image_paths_by_timestamp(root_dir: str) -> Dict[int, Dict[str, str]]:
# # # # #     """
# # # # #     读取三个文件夹的图片，按时间戳分组，记录各优先级路径
# # # # #     返回格式: {timestamp: {'draw': path, 'ori': path, 'raw': path}}
# # # # #     """
# # # # #     # 定义三个文件夹路径
# # # # #     raw_dir = os.path.join(root_dir, "images/raw")
# # # # #     draw_dir = os.path.join(root_dir, "images/detected/draw")
# # # # #     ori_dir = os.path.join(root_dir, "images/detected/ori")
    
# # # # #     # 初始化结果字典
# # # # #     timestamp_dict = {}
    
# # # # #     # 1. 读取raw文件夹
# # # # #     if os.path.exists(raw_dir):
# # # # #         for filename in os.listdir(raw_dir):
# # # # #             if filename.lower().endswith(('.jpg', '.jpeg', '.png')):
# # # # #                 ts = get_timestamp_from_filename(filename)
# # # # #                 if ts == 0:
# # # # #                     continue
# # # # #                 if ts not in timestamp_dict:
# # # # #                     timestamp_dict[ts] = {'draw': '', 'ori': '', 'raw': ''}
# # # # #                 timestamp_dict[ts]['raw'] = os.path.join(raw_dir, filename)
    
# # # # #     # 2. 读取ori文件夹
# # # # #     if os.path.exists(ori_dir):
# # # # #         for filename in os.listdir(ori_dir):
# # # # #             if filename.lower().endswith(('.jpg', '.jpeg', '.png')):
# # # # #                 ts = get_timestamp_from_filename(filename)
# # # # #                 if ts == 0:
# # # # #                     continue
# # # # #                 if ts not in timestamp_dict:
# # # # #                     timestamp_dict[ts] = {'draw': '', 'ori': '', 'raw': ''}
# # # # #                 timestamp_dict[ts]['ori'] = os.path.join(ori_dir, filename)
    
# # # # #     # 3. 读取draw文件夹（最高优先级）
# # # # #     if os.path.exists(draw_dir):
# # # # #         for filename in os.listdir(draw_dir):
# # # # #             if filename.lower().endswith(('.jpg', '.jpeg', '.png')):
# # # # #                 ts = get_timestamp_from_filename(filename)
# # # # #                 if ts == 0:
# # # # #                     continue
# # # # #                 if ts not in timestamp_dict:
# # # # #                     timestamp_dict[ts] = {'draw': '', 'ori': '', 'raw': ''}
# # # # #                 timestamp_dict[ts]['draw'] = os.path.join(draw_dir, filename)
    
# # # # #     return timestamp_dict

# # # # # def calculate_frame_interval(timestamps: List[int], default_fps: int = 30) -> Tuple[float, List[float]]:
# # # # #     """
# # # # #     根据时间戳计算每帧的间隔时间（秒）
# # # # #     :param timestamps: 排序后的时间戳列表（纳秒）
# # # # #     :param default_fps: 默认帧率（无时间差时使用）
# # # # #     :return: 平均帧率, 每帧间隔时间列表
# # # # #     """
# # # # #     if len(timestamps) < 2:
# # # # #         return default_fps, [1.0/default_fps] * len(timestamps)
    
# # # # #     # 转换时间戳为秒，并计算相邻帧的时间差
# # # # #     timestamps_sec = [ts / 1e9 for ts in timestamps]
# # # # #     frame_intervals = []
    
# # # # #     for i in range(1, len(timestamps_sec)):
# # # # #         interval = timestamps_sec[i] - timestamps_sec[i-1]
# # # # #         # 过滤异常值（间隔小于0或大于1秒的按默认处理）
# # # # #         if interval <= 0 or interval > 1.0:
# # # # #             interval = 1.0 / default_fps
# # # # #         frame_intervals.append(interval)
    
# # # # #     # 第一帧用默认间隔
# # # # #     frame_intervals.insert(0, 1.0 / default_fps)
    
# # # # #     # 计算平均帧率
# # # # #     avg_interval = np.mean(frame_intervals)
# # # # #     avg_fps = 1.0 / avg_interval if avg_interval > 0 else default_fps
    
# # # # #     return avg_fps, frame_intervals

# # # # # def create_video_from_images(root_dir: str, output_video_path: str, default_fps: int = 30):
# # # # #     """
# # # # #     主函数：生成视频
# # # # #     :param root_dir: 根目录（包含images文件夹）
# # # # #     :param output_video_path: 输出视频路径（如：output.mp4）
# # # # #     :param default_fps: 默认帧率
# # # # #     """
# # # # #     # 1. 获取按时间戳分组的图片路径
# # # # #     print("读取图片路径...")
# # # # #     timestamp_dict = get_image_paths_by_timestamp(root_dir)
# # # # #     if not timestamp_dict:
# # # # #         print("未找到任何有效图片！")
# # # # #         return
    
# # # # #     # 2. 按时间戳排序
# # # # #     sorted_timestamps = sorted(timestamp_dict.keys())
# # # # #     print(f"共找到 {len(sorted_timestamps)} 个时间戳的图片")
    
# # # # #     # 3. 计算帧间隔和平均帧率
# # # # #     avg_fps, frame_intervals = calculate_frame_interval(sorted_timestamps, default_fps)
# # # # #     print(f"平均帧率: {avg_fps:.2f} FPS")
    
# # # # #     # 4. 获取第一张有效图片的尺寸（作为视频分辨率）
# # # # #     first_img_path = ""
# # # # #     for ts in sorted_timestamps:
# # # # #         # 按优先级选择：draw > ori > raw
# # # # #         if timestamp_dict[ts]['draw']:
# # # # #             first_img_path = timestamp_dict[ts]['draw']
# # # # #         elif timestamp_dict[ts]['ori']:
# # # # #             first_img_path = timestamp_dict[ts]['ori']
# # # # #         elif timestamp_dict[ts]['raw']:
# # # # #             first_img_path = timestamp_dict[ts]['raw']
        
# # # # #         if first_img_path:
# # # # #             break
    
# # # # #     if not first_img_path:
# # # # #         print("未找到有效图片！")
# # # # #         return
    
# # # # #     first_img = cv2.imread(first_img_path)
# # # # #     if first_img is None:
# # # # #         print(f"无法读取图片: {first_img_path}")
# # # # #         return
# # # # #     height, width = first_img.shape[:2]
# # # # #     print(f"视频分辨率: {width}x{height}")
    
# # # # #     # 5. 初始化视频写入器
# # # # #     fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # MP4格式
# # # # #     video_writer = cv2.VideoWriter(
# # # # #         output_video_path,
# # # # #         fourcc,
# # # # #         avg_fps,  # 使用平均帧率
# # # # #         (width, height)
# # # # #     )
    
# # # # #     if not video_writer.isOpened():
# # # # #         print("无法创建视频写入器！")
# # # # #         return
    
# # # # #     # 6. 逐帧写入视频
# # # # #     print("开始生成视频...")
# # # # #     for idx, ts in enumerate(sorted_timestamps):
# # # # #         # 按优先级选择图片：draw > ori > raw
# # # # #         img_path = ""
# # # # #         if timestamp_dict[ts]['draw']:
# # # # #             img_path = timestamp_dict[ts]['draw']
# # # # #         elif timestamp_dict[ts]['ori']:
# # # # #             img_path = timestamp_dict[ts]['ori']
# # # # #         elif timestamp_dict[ts]['raw']:
# # # # #             img_path = timestamp_dict[ts]['raw']
        
# # # # #         if not img_path:
# # # # #             print(f"时间戳 {ts} 无有效图片，跳过")
# # # # #             continue
        
# # # # #         # 读取图片
# # # # #         img = cv2.imread(img_path)
# # # # #         if img is None:
# # # # #             print(f"无法读取图片 {img_path}，跳过")
# # # # #             continue
        
# # # # #         # 确保图片尺寸与视频一致
# # # # #         if img.shape[1] != width or img.shape[0] != height:
# # # # #             img = cv2.resize(img, (width, height))
        
# # # # #         # 写入视频（根据时间间隔决定写入多少帧，保证时间同步）
# # # # #         frame_duration = frame_intervals[idx]
# # # # #         frame_count = max(1, int(round(frame_duration * avg_fps)))  # 至少写入1帧
# # # # #         for _ in range(frame_count):
# # # # #             video_writer.write(img)
        
# # # # #         # 进度提示
# # # # #         if (idx + 1) % 100 == 0:
# # # # #             print(f"已处理 {idx + 1}/{len(sorted_timestamps)} 帧")
    
# # # # #     # 7. 释放资源
# # # # #     video_writer.release()
# # # # #     cv2.destroyAllWindows()
# # # # #     print(f"视频生成完成！保存路径: {output_video_path}")

# # # # # if __name__ == "__main__":
# # # # #     # ==================== 配置参数 ====================
# # # # #     ROOT_DIR = "/home/bsa/A_vision_relate/data_20260126_114314/"  # 替换为你的根目录（包含images文件夹）
# # # # #     OUTPUT_VIDEO_PATH = "output_video.mp4"     # 输出视频路径
# # # # #     DEFAULT_FPS = 30                           # 默认帧率
# # # # #     # ==================================================
    
# # # # #     create_video_from_images(ROOT_DIR, OUTPUT_VIDEO_PATH, DEFAULT_FPS)


# # # # import os
# # # # import cv2
# # # # import numpy as np
# # # # from pathlib import Path

# # # # def get_timestamp_from_filename(filename: str) -> int:
# # # #     """从文件名提取纳秒时间戳"""
# # # #     try:
# # # #         return int(os.path.splitext(filename)[0])
# # # #     except ValueError:
# # # #         return 0

# # # # def get_priority_image_paths(root_dir: str) -> list:
# # # #     """
# # # #     按优先级（draw>ori>raw）获取排序后的图片路径列表
# # # #     返回：[(timestamp, img_path), ...]
# # # #     """
# # # #     # 定义文件夹路径
# # # #     raw_dir = os.path.join(root_dir, "images/raw")
# # # #     draw_dir = os.path.join(root_dir, "images/detected/draw")
# # # #     ori_dir = os.path.join(root_dir, "images/detected/ori")
    
# # # #     # 第一步：收集所有时间戳和对应路径
# # # #     ts_dict = {}
    
# # # #     # 先收集raw（最低优先级，会被后续覆盖）
# # # #     if os.path.exists(raw_dir):
# # # #         for f in os.listdir(raw_dir):
# # # #             if f.lower().endswith(('.jpg', '.jpeg', '.png')):
# # # #                 ts = get_timestamp_from_filename(f)
# # # #                 if ts > 0:
# # # #                     ts_dict[ts] = os.path.join(raw_dir, f)
    
# # # #     # 再收集ori（覆盖raw）
# # # #     if os.path.exists(ori_dir):
# # # #         for f in os.listdir(ori_dir):
# # # #             if f.lower().endswith(('.jpg', '.jpeg', '.png')):
# # # #                 ts = get_timestamp_from_filename(f)
# # # #                 if ts > 0:
# # # #                     ts_dict[ts] = os.path.join(ori_dir, f)
    
# # # #     # 最后收集draw（最高优先级，覆盖ori/raw）
# # # #     if os.path.exists(draw_dir):
# # # #         for f in os.listdir(draw_dir):
# # # #             if f.lower().endswith(('.jpg', '.jpeg', '.png')):
# # # #                 ts = get_timestamp_from_filename(f)
# # # #                 if ts > 0:
# # # #                     ts_dict[ts] = os.path.join(draw_dir, f)
    
# # # #     # 第二步：按时间戳排序
# # # #     sorted_items = sorted(ts_dict.items(), key=lambda x: x[0])
# # # #     # 提取路径列表
# # # #     result = [(ts, path) for ts, path in sorted_items if cv2.imread(path) is not None]
    
# # # #     return result

# # # # def create_playable_video(root_dir: str, output_path: str, fixed_fps: int = 10):
# # # #     """
# # # #     创建可播放的视频（固定帧率，通用编码）
# # # #     :param root_dir: 根目录（包含images文件夹）
# # # #     :param output_path: 输出视频路径（如 output.mp4）
# # # #     :param fixed_fps: 固定帧率（建议10-30，越低越不易出问题）
# # # #     """
# # # #     # 1. 获取排序后的图片路径
# # # #     print("读取图片列表...")
# # # #     img_list = get_priority_image_paths(root_dir)
# # # #     if not img_list:
# # # #         print("错误：未找到任何有效图片！")
# # # #         return
# # # #     print(f"共找到 {len(img_list)} 张有效图片")
    
# # # #     # 2. 确定视频分辨率（统一缩放为第一张图的尺寸）
# # # #     first_ts, first_path = img_list[0]
# # # #     first_img = cv2.imread(first_path)
# # # #     if first_img is None:
# # # #         print(f"错误：无法读取第一张图片 {first_path}")
# # # #         return
# # # #     H, W = first_img.shape[:2]
# # # #     print(f"视频分辨率：{W}x{H}，帧率：{fixed_fps} FPS")
    
# # # #     # 3. 初始化视频写入器（改用更兼容的编码）
# # # #     # 方案1：mp4通用编码（推荐）
# # # #     fourcc = cv2.VideoWriter_fourcc(*'avc1')  # H.264编码，兼容性最好
# # # #     # 方案2：如果avc1不行，改用mp4v（备用）
# # # #     # fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    
# # # #     # 检查输出路径目录是否存在
# # # #     output_dir = os.path.dirname(output_path)
# # # #     if output_dir and not os.path.exists(output_dir):
# # # #         os.makedirs(output_dir)
    
# # # #     # 创建写入器
# # # #     video_writer = cv2.VideoWriter(
# # # #         output_path,
# # # #         fourcc,
# # # #         fixed_fps,
# # # #         (W, H)
# # # #     )
    
# # # #     if not video_writer.isOpened():
# # # #         print("错误：无法创建视频写入器！请检查：")
# # # #         print("  1. 输出路径是否有权限（如：/root/ 需sudo）")
# # # #         print("  2. 编码格式是否支持（尝试切换fourcc为mp4v）")
# # # #         return
    
# # # #     # 4. 逐帧写入（统一缩放尺寸）
# # # #     print("开始生成视频...")
# # # #     for idx, (ts, img_path) in enumerate(img_list):
# # # #         # 读取图片
# # # #         img = cv2.imread(img_path)
# # # #         if img is None:
# # # #             print(f"警告：跳过无法读取的图片 {img_path}")
# # # #             continue
        
# # # #         # 强制缩放为统一尺寸
# # # #         if img.shape[1] != W or img.shape[0] != H:
# # # #             img = cv2.resize(img, (W, H), interpolation=cv2.INTER_LINEAR)
        
# # # #         # 写入视频
# # # #         video_writer.write(img)
        
# # # #         # 进度提示
# # # #         if (idx + 1) % 50 == 0:
# # # #             print(f"进度：{idx+1}/{len(img_list)}")
    
# # # #     # 5. 释放资源
# # # #     video_writer.release()
# # # #     cv2.destroyAllWindows()
# # # #     print(f"\n视频生成完成！路径：{output_path}")
# # # #     print("建议使用：VLC播放器、PotPlayer 或 浏览器 打开（兼容性最好）")

# # # # if __name__ == "__main__":
# # # #     # ==================== 请修改这两个参数 ====================
# # # #     ROOT_DIR = "/home/bsa/A_vision_relate/data_20260126_114314/"  # 替换为你的根目录（含images文件夹）
# # # #     OUTPUT_VIDEO = "output_playable.mp4" # 输出视频路径
# # # #     # =========================================================
    
# # # #     create_playable_video(ROOT_DIR, OUTPUT_VIDEO, fixed_fps=10)



# # # import cv2
# # # import os
# # # import glob
# # # import numpy as np
# # # from pathlib import Path

# # # def create_video_from_timestamps(base_dir, output_name='output.mp4', target_fps=30):
# # #     """
# # #     base_dir: 包含 images 文件夹的根目录
# # #     target_fps: 视频播放的基准帧率
# # #     """
    
# # #     # 1. 定义路径
# # #     paths = {
# # #         'draw': os.path.join(base_dir, 'images/detected/draw'),
# # #         'ori': os.path.join(base_dir, 'images/detected/ori'),
# # #         'raw': os.path.join(base_dir, 'images/raw')
# # #     }

# # #     # 2. 优先级合并 (draw > ori > raw)
# # #     # 使用字典: { timestamp: full_path }
# # #     ts_map = {}

# # #     # 按优先级从低到高读取，高优先级会覆盖低优先级
# # #     for folder_type in ['raw', 'ori', 'draw']:
# # #         folder_path = paths[folder_type]
# # #         if not os.path.exists(folder_path):
# # #             print(f"Warning: Folder {folder_path} not found.")
# # #             continue
            
# # #         for img_path in glob.glob(os.path.join(folder_path, "*.jpg")):
# # #             ts_str = Path(img_path).stem  # 获取文件名（不带后缀的时间戳）
# # #             try:
# # #                 ts = int(ts_str)
# # #                 ts_map[ts] = img_path
# # #             except ValueError:
# # #                 continue

# # #     if not ts_map:
# # #         print("No images found!")
# # #         return

# # #     # 3. 按时间戳排序
# # #     sorted_ts = sorted(ts_map.keys())
    
# # #     # 4. 初始化视频写入器
# # #     # 读取第一张图确定宽高
# # #     first_img = cv2.imread(ts_map[sorted_ts[0]])
# # #     h, w, _ = first_img.shape
    
# # #     fourcc = cv2.VideoWriter_fourcc(*'mp4v') # 或 'X264'
# # #     video_writer = cv2.VideoWriter(output_name, fourcc, target_fps, (w, h))

# # #     print(f"Total unique timestamps: {len(sorted_ts)}")
# # #     print("Processing frames...")

# # #     # 5. 写入视频
# # #     # 如果你想实现“动态帧率”感知真实时间：
# # #     # 我们计算相邻两帧的时间差，如果差值过大，可以重复写入同一帧（抽帧补帧逻辑）
    
# # #     # 计算平均间隔（纳秒转秒）作为参考
# # #     # 若不考虑真实间隔，直接设置 real_time_mode = False
# # #     real_time_mode = True 
# # #     time_scale = 1.0  # 播放速度倍数，1.0为原速
    
# # #     for i in range(len(sorted_ts)):
# # #         ts = sorted_ts[i]
# # #         img_path = ts_map[ts]
# # #         frame = cv2.imread(img_path)

# # #         if frame is None:
# # #             continue
        
# # #         # 调整尺寸对齐第一张图（防止文件夹内图片尺寸不一）
# # #         if frame.shape[0] != h or frame.shape[1] != w:
# # #             frame = cv2.resize(frame, (w, h))

# # #         if not real_time_mode or i == len(sorted_ts) - 1:
# # #             video_writer.write(frame)
# # #         else:
# # #             # 计算当前帧到下一帧的持续时间 (秒)
# # #             duration_ns = sorted_ts[i+1] - ts
# # #             duration_sec = duration_ns / 1e9
            
# # #             # 计算在 target_fps 下需要写入多少次这一帧
# # #             # 例如：间隔0.1s，FPS=30，则需要写入 0.1 * 30 = 3 帧
# # #             num_repeats = max(1, int(round(duration_sec * target_fps / time_scale)))
            
# # #             # 限制过度补帧（防止时间戳断层导致视频卡死）
# # #             if num_repeats > target_fps * 2: 
# # #                 num_repeats = target_fps * 2
                
# # #             for _ in range(num_repeats):
# # #                 video_writer.write(frame)

# # #         if i % 100 == 0:
# # #             print(f"Processed {i}/{len(sorted_ts)} frames...")

# # #     video_writer.release()
# # #     print(f"Video saved as {output_name}")

# # # # --- 使用示例 ---
# # # if __name__ == "__main__":
# # #     # 修改为你的实际路径
# # #     # 假设结构是 /path/to/data/images/raw 等
# # #     my_base_dir = "/home/bsa/A_vision_relate/data_20260126_114314/" 
# # #     create_video_from_timestamps(my_base_dir, "result_video.mp4", target_fps=30)


# # import cv2
# # import os
# # import glob
# # import numpy as np
# # import pandas as pd
# # from pathlib import Path
# # from bisect import bisect_left

# # def get_closest_data(ts, df):
# #     """在 DataFrame 中找到最接近时间戳的一行数据"""
# #     if df is None or df.empty:
# #         return None
# #     # 找到插入位置
# #     idx = bisect_left(df['timestamp_ns'].values, ts)
# #     if idx == 0:
# #         return df.iloc[0]
# #     if idx >= len(df):
# #         return df.iloc[-1]
# #     # 比较前后两个哪个更近
# #     before = df.iloc[idx - 1]
# #     after = df.iloc[idx]
# #     if ts - before['timestamp_ns'] < after['timestamp_ns'] - ts:
# #         return before
# #     else:
# #         return after

# # def create_debug_video(base_dir, output_name='flight_debug.mp4', target_fps=30):
# #     # 1. 路径定义
# #     img_paths = {
# #         'draw': os.path.join(base_dir, 'images/detected/draw'),
# #         'ori': os.path.join(base_dir, 'images/detected/ori'),
# #         'raw': os.path.join(base_dir, 'images/raw')
# #     }
    
# #     # 2. 加载 CSV 状态数据
# #     try:
# #         uav_df = pd.read_csv(os.path.join(base_dir, 'uav_states.csv')).sort_values('timestamp_ns')
# #         tgt_df = pd.read_csv(os.path.join(base_dir, 'target_states.csv')).sort_values('timestamp_ns')
# #         gim_df = pd.read_csv(os.path.join(base_dir, 'gimbal_states.csv')).sort_values('timestamp_ns')
# #         print("CSV states loaded successfully.")
# #     except Exception as e:
# #         print(f"Error loading CSVs: {e}")
# #         return

# #     # 3. 优先级合并图片 (draw > ori > raw)
# #     ts_map = {}
# #     for folder_type in ['raw', 'ori', 'draw']:
# #         path = img_paths[folder_type]
# #         if not os.path.exists(path): continue
# #         for f in glob.glob(os.path.join(path, "*.jpg")):
# #             ts_map[int(Path(f).stem)] = f

# #     sorted_ts = sorted(ts_map.keys())
# #     if not sorted_ts:
# #         print("No images found!"); return

# #     start_time_ns = sorted_ts[0] # 以第一张图或UAV第一条数据为基准

# #     # 4. 初始化视频写入器
# #     # 获取图片尺寸并增加侧边栏宽度 (例如原图宽度的 1/3)
# #     sample_img = cv2.imread(ts_map[sorted_ts[0]])
# #     img_h, img_w = sample_img.shape[:2]
# #     sidebar_w = 400  # 固定侧边栏宽度
# #     total_w = img_w + sidebar_w
    
# #     # 使用 'mp4v' 或 'avc1'，后缀用 .mp4
# #     fourcc = cv2.VideoWriter_fourcc(*'mp4v') 
# #     video_writer = cv2.VideoWriter(output_name, fourcc, target_fps, (total_w, img_h))

# #     print(f"Processing {len(sorted_ts)} frames...")

# #     for i in range(len(sorted_ts)):
# #         ts = sorted_ts[i]
# #         img = cv2.imread(ts_map[ts])
# #         if img is None: continue
# #         if img.shape[0] != img_h or img.shape[1] != img_w:
# #             img = cv2.resize(img, (img_w, img_h))

# #         # 创建黑色画布
# #         canvas = np.zeros((img_h, total_w, 3), dtype=np.uint8)
# #         canvas[:, :img_w] = img # 左边放原图

# #         # 5. 获取并绘制遥测数据
# #         uav = get_closest_data(ts, uav_df)
# #         tgt = get_closest_data(ts, tgt_df)
# #         gim = get_closest_data(ts, gim_df)

# #         runtime = (ts - start_time_ns) / 1e9
        
# #         info_text = [
# #             f"Runtime: {runtime:.2f} s",
# #             "",
# #             "[UAV State]",
# #             f"Pos: {uav['position_x']:.2f}, {uav['position_y']:.2f}, {uav['position_z']:.2f}",
# #             f"Att: R:{uav['attitude_roll']:.2f} P:{uav['attitude_pitch']:.2f} Y:{uav['attitude_yaw']:.2f}",
# #             "",
# #             "[Target State]",
# #             f"Pos: {tgt['px']:.2f}, {tgt['py']:.2f}, {tgt['pz']:.2f}",
# #             f"Att: {tgt['attitude0']:.2f}, {tgt['attitude1']:.2f}, {tgt['attitude2']:.2f}",
# #             "",
# #             "[Gimbal/Camera]",
# #             f"Pitch: {gim['pitch']:.2f} Yaw: {gim['yaw']:.2f}",
# #             f"Zoom: {gim['zoom']:.2f}",
# #         ]

# #         # 绘制文字到侧边栏
# #         y0, dy = 40, 30
# #         for j, line in enumerate(info_text):
# #             color = (255, 255, 255) if not line.startswith("[") else (0, 255, 255)
# #             cv2.putText(canvas, line, (img_w + 20, y0 + j*dy), 
# #                         cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 1, cv2.LINE_AA)

# #         # 6. 动态帧率处理 (重复写入帧)
# #         if i < len(sorted_ts) - 1:
# #             duration_sec = (sorted_ts[i+1] - ts) / 1e9
# #             num_repeats = max(1, int(round(duration_sec * target_fps)))
# #             if num_repeats > target_fps * 2: num_repeats = target_fps * 2 # 避免断层卡死
# #         else:
# #             num_repeats = 1

# #         for _ in range(num_repeats):
# #             video_writer.write(canvas)

# #         if i % 100 == 0:
# #             print(f"Frame {i}/{len(sorted_ts)} written.")

# #     video_writer.release()
# #     print(f"Done! Video saved as {output_name}")

# # if __name__ == "__main__":
# #     # 确保此目录下有 images 文件夹和那几个 .csv 文件
# #     create_debug_video(base_dir="/home/bsa/A_vision_relate/data_20260126_114314/")


# import cv2
# import os
# import glob
# import numpy as np
# import pandas as pd
# from pathlib import Path
# from bisect import bisect_left

# def get_closest_data(ts, df):
#     """在 DataFrame 中找到最接近时间戳的一行数据"""
#     if df is None or df.empty:
#         return None
#     # 找到插入位置
#     idx = bisect_left(df['timestamp_ns'].values, ts)
#     if idx == 0:
#         return df.iloc[0]
#     if idx >= len(df):
#         return df.iloc[-1]
#     # 比较前后两个哪个更近
#     before = df.iloc[idx - 1]
#     after = df.iloc[idx]
#     if ts - before['timestamp_ns'] < after['timestamp_ns'] - ts:
#         return before
#     else:
#         return after

# def create_debug_video(base_dir, output_name='flight_debug.mp4', target_fps=30):
#     # 1. 路径定义
#     img_paths = {
#         'draw': os.path.join(base_dir, 'images/detected/draw'),
#         'ori': os.path.join(base_dir, 'images/detected/ori'),
#         'raw': os.path.join(base_dir, 'images/raw')
#     }
    
#     # 2. 加载 CSV 状态数据
#     try:
#         uav_df = pd.read_csv(os.path.join(base_dir, 'uav_states.csv')).sort_values('timestamp_ns')
#         tgt_df = pd.read_csv(os.path.join(base_dir, 'target_states.csv')).sort_values('timestamp_ns')
#         gim_df = pd.read_csv(os.path.join(base_dir, 'gimbal_states.csv')).sort_values('timestamp_ns')
#         print("CSV states loaded successfully.")
#     except Exception as e:
#         print(f"Error loading CSVs: {e}")
#         return

#     # 3. 优先级合并图片 (draw > ori > raw)
#     ts_map = {}
#     for folder_type in ['raw', 'ori', 'draw']:
#         path = img_paths[folder_type]
#         if not os.path.exists(path): continue
#         for f in glob.glob(os.path.join(path, "*.jpg")):
#             ts_map[int(Path(f).stem)] = f

#     sorted_ts = sorted(ts_map.keys())
#     if not sorted_ts:
#         print("No images found!"); return

#     start_time_ns = sorted_ts[0] # 以第一张图或UAV第一条数据为基准

#     # 4. 初始化视频写入器
#     # 获取图片尺寸并增加侧边栏宽度 (例如原图宽度的 1/3)
#     sample_img = cv2.imread(ts_map[sorted_ts[0]])
#     img_h, img_w = sample_img.shape[:2]
#     sidebar_w = 400  # 固定侧边栏宽度
#     total_w = img_w + sidebar_w
    
#     # 使用 'mp4v' 或 'avc1'，后缀用 .mp4
#     fourcc = cv2.VideoWriter_fourcc(*'mp4v') 
#     video_writer = cv2.VideoWriter(output_name, fourcc, target_fps, (total_w, img_h))

#     print(f"Processing {len(sorted_ts)} frames...")

#     for i in range(len(sorted_ts)):
#         ts = sorted_ts[i]
#         img = cv2.imread(ts_map[ts])
#         if img is None: continue
#         if img.shape[0] != img_h or img.shape[1] != img_w:
#             img = cv2.resize(img, (img_w, img_h))

#         # 创建黑色画布
#         canvas = np.zeros((img_h, total_w, 3), dtype=np.uint8)
#         canvas[:, :img_w] = img # 左边放原图

#         # 5. 获取并绘制遥测数据
#         uav = get_closest_data(ts, uav_df)
#         tgt = get_closest_data(ts, tgt_df)
#         gim = get_closest_data(ts, gim_df)

#         runtime = (ts - start_time_ns) / 1e9
        
#         info_text = [
#             f"Runtime: {runtime:.2f} s",
#             "",
#             "[UAV State]",
#             f"Pos: {uav['position_x']:.2f}, {uav['position_y']:.2f}, {uav['position_z']:.2f}",
#             f"Att: R:{uav['attitude_roll']:.2f} P:{uav['attitude_pitch']:.2f} Y:{uav['attitude_yaw']:.2f}",
#             "",
#             "[Target State]",
#             f"Pos: {tgt['px']:.2f}, {tgt['py']:.2f}, {tgt['pz']:.2f}",
#             f"Att: {tgt['attitude0']:.2f}, {tgt['attitude1']:.2f}, {tgt['attitude2']:.2f}",
#             "",
#             "[Gimbal/Camera]",
#             f"Pitch: {gim['pitch']:.2f} Yaw: {gim['yaw']:.2f}",
#             f"Zoom: {gim['zoom']:.2f}",
#         ]

#         # 绘制文字到侧边栏
#         y0, dy = 40, 30
#         for j, line in enumerate(info_text):
#             color = (255, 255, 255) if not line.startswith("[") else (0, 255, 255)
#             cv2.putText(canvas, line, (img_w + 20, y0 + j*dy), 
#                         cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 1, cv2.LINE_AA)

#         # 6. 动态帧率处理 (重复写入帧)
#         if i < len(sorted_ts) - 1:
#             duration_sec = (sorted_ts[i+1] - ts) / 1e9
#             num_repeats = max(1, int(round(duration_sec * target_fps)))
#             if num_repeats > target_fps * 2: num_repeats = target_fps * 2 # 避免断层卡死
#         else:
#             num_repeats = 1

#         for _ in range(num_repeats):
#             video_writer.write(canvas)

#         if i % 100 == 0:
#             print(f"Frame {i}/{len(sorted_ts)} written.")

#     video_writer.release()
#     print(f"Done! Video saved as {output_name}")

# if __name__ == "__main__":
#     # 确保此目录下有 images 文件夹和那几个 .csv 文件
#     create_debug_video(base_dir="/home/bsa/A_vision_relate/data_20260126_114314/")

import cv2
import os
import glob
import numpy as np
import pandas as pd
from pathlib import Path
from bisect import bisect_left

def get_closest_data(ts, df):
    """高效查找最接近时间戳的 CSV 数据行"""
    if df is None or df.empty:
        return None
    ts_array = df['timestamp_ns'].values
    idx = bisect_left(ts_array, ts)
    if idx == 0: return df.iloc[0]
    if idx >= len(df): return df.iloc[-1]
    before = df.iloc[idx - 1]
    after = df.iloc[idx]
    return before if (ts - before['timestamp_ns'] < after['timestamp_ns'] - ts) else after

def create_debug_video(base_dir, output_name='flight_analysis.mp4', target_fps=30):
    # 1. 加载数据
    try:
        uav_df = pd.read_csv(os.path.join(base_dir, 'uav_states.csv')).sort_values('timestamp_ns')
        tgt_df = pd.read_csv(os.path.join(base_dir, 'target_states.csv')).sort_values('timestamp_ns')
        gim_df = pd.read_csv(os.path.join(base_dir, 'gimbal_states.csv')).sort_values('timestamp_ns')
        print("CSVs loaded.")
    except Exception as e:
        print(f"Error: {e}"); return

    # 2. 优先级合并图片 (draw > ori > raw)
    img_paths = {'draw': 'images/detected/draw', 'ori': 'images/detected/ori', 'raw': 'images/raw'}
    ts_map = {}
    for level, rel_path in img_paths.items():
        p = os.path.join(base_dir, rel_path)
        if not os.path.exists(p): continue
        for f in glob.glob(os.path.join(p, "*.jpg")):
            ts_map[int(Path(f).stem)] = f

    sorted_img_ts = sorted(ts_map.keys())
    if not sorted_img_ts: return

    # 3. 视频初始化
    sample_img = cv2.imread(ts_map[sorted_img_ts[0]])
    h, w = sample_img.shape[:2]
    sidebar_w = 480
    # 尝试多种编码器以确保兼容性
    fourcc = cv2.VideoWriter_fourcc(*'mp4v') 
    out = cv2.VideoWriter(output_name, fourcc, target_fps, (w + sidebar_w, h))

    # 4. 时间轴逻辑
    start_ts = sorted_img_ts[0]
    end_ts = sorted_img_ts[-1]
    frame_duration_ns = int(1e9 / target_fps)
    
    current_video_ts = start_ts
    img_idx = 0
    
    print("Generating frames...")
    
    # 以时间轴为主循环
    while current_video_ts <= end_ts:
        # A. 确定当前应该显示的图片 (寻找不大于当前视频时间的最新图片)
        while img_idx + 1 < len(sorted_img_ts) and sorted_img_ts[img_idx+1] <= current_video_ts:
            img_idx += 1
        
        curr_img_ts = sorted_img_ts[img_idx]
        img = cv2.imread(ts_map[curr_img_ts])
        if img is None: break
        if img.shape[0] != h or img.shape[1] != w:
            img = cv2.resize(img, (w, h))

        # B. 准备画布
        canvas = np.zeros((h, w + sidebar_w, 3), dtype=np.uint8)
        canvas[:, :w] = img
        
        # C. 获取数据
        # 1. 同步数据 (Synced with Image)
        uav_s = get_closest_data(curr_img_ts, uav_df)
        tgt_s = get_closest_data(curr_img_ts, tgt_df)
        gim_s = get_closest_data(curr_img_ts, gim_df)
        
        # 2. 实时数据 (Real-time with Video Timeline)
        uav_r = get_closest_data(current_video_ts, uav_df)
        gim_r = get_closest_data(current_video_ts, gim_df)

        # D. 绘制 UI
        runtime = (current_video_ts - start_ts) / 1e9
        y_pos = 30
        def draw_t(text, color=(255,255,255), size=0.5, thick=1):
            nonlocal y_pos
            cv2.putText(canvas, text, (w + 20, y_pos), cv2.FONT_HERSHEY_SIMPLEX, size, color, thick, cv2.LINE_AA)
            y_pos += 25

        draw_t(f"RUNTIME: {runtime:.2f} s", (0, 255, 0), 0.7, 2)
        draw_t("-" * 40)
        
        # 图像同步数据块
        draw_t("[ SYNCED STATE (FROM IMG) ]", (0, 255, 255), 0.6, 2)
        draw_t(f"Img TS: {curr_img_ts}")
        draw_t(f"UAV P: {uav_s['position_x']:.2f}, {uav_s['position_y']:.2f}, {uav_s['position_z']:.2f}")
        draw_t(f"UAV A: R:{uav_s['attitude_roll']:.2f} P:{uav_s['attitude_pitch']:.2f} Y:{uav_s['attitude_yaw']:.2f}")
        draw_t(f"TGT P: {tgt_s['px']:.2f}, {tgt_s['py']:.2f}, {tgt_s['pz']:.2f}")
        draw_t(f"GIM A: P:{gim_s['pitch']:.2f} Y:{gim_s['yaw']:.2f}")
        
        y_pos += 20
        draw_t("-" * 40)
        
        # 实时数据块 (对比观察)
        draw_t("[ REAL-TIME STATE (LIVE) ]", (255, 100, 100), 0.6, 2)
        draw_t(f"Live TS: {current_video_ts}")
        draw_t(f"UAV P: {uav_r['position_x']:.2f}, {uav_r['position_y']:.2f}, {uav_r['position_z']:.2f}")
        draw_t(f"UAV A: R:{uav_r['attitude_roll']:.2f} P:{uav_r['attitude_pitch']:.2f} Y:{uav_r['attitude_yaw']:.2f}")
        draw_t(f"GIM A: P:{gim_r['pitch']:.2f} Y:{gim_r['yaw']:.2f} Z:{gim_r['zoom']:.1f}")
        
        # E. 写入视频并推进时间
        out.write(canvas)
        current_video_ts += frame_duration_ns
        
        if img_idx % 100 == 0:
            print(f"Time: {runtime:.2f}s | Image Index: {img_idx}")

    out.release()
    print(f"Finished! Video: {output_name}")

if __name__ == "__main__":
    create_debug_video("/home/bsa/A_vision_relate/data_20260126_114314/")
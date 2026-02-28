import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os
import struct
import shutil
from matplotlib.backends.backend_pdf import PdfPages
from scipy.spatial.transform import Rotation as R  # 用于准确计算欧拉角

# ================= 配置区域 =================
DATA_DIR = "/home/bsa/A_vision_relate/0124_07_原地无人机旋转_云台锁定-fupan"

# C++ 模型参数
MODEL_CONFIG = {
    'seq_len': 300,      # 输入历史长度 (Input Sequence Length)
    'pred_len': 50,      # 输出预测长度 (Prediction Sequence Length)
    'n_vars': 6,         # 特征数量 (x,y,z, vx,vy,vz, ax,ay,az...)
    'feature_names': ['Pos X', 'Pos Y', 'Pos Z', 
                      'euler roll', 'euler pitch', 'euler yaw'] # 名字需与 n_vars 对应
}
FREQ_HZ = 10

# ================= 绘图设置 =================
plt.rcParams['font.sans-serif'] = ['DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False
# 图片尺寸设大一点，以容纳 4x3 的布局
FIG_SIZE = (20, 16) 

# ================= 第一部分：CSV 轨迹对比 (增强版) =================

def calculate_euler_from_quat(df):
    """从四元数计算欧拉角 (ZYX顺序, 输出为度)"""
    required = ['qx', 'qy', 'qz', 'qw']
    if not all(col in df.columns for col in required):
        return df
    
    try:
        # scipy 的 Rotation对象
        r = R.from_quat(df[['qx', 'qy', 'qz', 'qw']].values)
        # 转换为欧拉角 (ZYX顺序: Yaw, Pitch, Roll)
        euler = r.as_euler('zyx', degrees=True)
        
        df['yaw_deg'] = euler[:, 0]
        df['pitch_deg'] = euler[:, 1]
        df['roll_deg'] = euler[:, 2]
        
        # 解卷绕 (Unwrap) 处理，防止 -180 到 180 的跳变影响观察
        df['yaw_deg'] = np.rad2deg(np.unwrap(np.deg2rad(df['yaw_deg'])))
        df['pitch_deg'] = np.rad2deg(np.unwrap(np.deg2rad(df['pitch_deg'])))
        df['roll_deg'] = np.rad2deg(np.unwrap(np.deg2rad(df['roll_deg'])))
        
        print(" -> 已从四元数计算欧拉角 (deg)")
    except Exception as e:
        print(f" -> 计算欧拉角失败: {e}")
        
    return df

def read_trajectory_csv(file_path, is_uav=False, base_timestamp=None):
    if not os.path.exists(file_path):
        print(f"[Warn] 文件不存在: {file_path}")
        return pd.DataFrame()

    try:
        df = pd.read_csv(file_path)
    except Exception as e:
        print(f"[Error] 读取文件失败 {file_path}: {e}")
        return pd.DataFrame()
    
    # 1. 基础列名映射 (位置)
    rename_dict = {}
    if is_uav:
        rename_dict = {
            'position_x': 'x', 'position_y': 'y', 'position_z': 'z', 
            'timestamp_ns': 'timestamp',
            'attitude_qx': 'qx', 'attitude_qy': 'qy', 'attitude_qz': 'qz', 'attitude_qw': 'qw',
            'attitude_roll': 'roll', 'attitude_pitch': 'pitch', 'attitude_yaw': 'yaw'
        }
    else:
        # 目标可能的列名 (适配不同版本的 log)
        rename_dict = {
            'px': 'x', 'py': 'y', 'pz': 'z', 
            'timestamp_ns': 'timestamp',
            # 适配 msg 中的 attitude_q
            'attitude_q_x': 'qx', 'attitude_q.x': 'qx', 
            'attitude_q_y': 'qy', 'attitude_q.y': 'qy',
            'attitude_q_z': 'qz', 'attitude_q.z': 'qz',
            'attitude_q_w': 'qw', 'attitude_q.w': 'qw',
            # 或者是 orientation
            'attitude0': 'roll', 
            'attitude1': 'pitch', 
            'attitude2': 'yaw'
        }
    
    # 执行重命名
    df.rename(columns={k: v for k, v in rename_dict.items() if k in df.columns}, inplace=True)
    
    # 检查必要列
    if 'timestamp' not in df.columns:
        return pd.DataFrame()
    
    # 2. 计算相对时间
    if base_timestamp is not None:
        df['time_s'] = (df['timestamp'] - base_timestamp) * 1e-9
    else:
        df['time_s'] = (df['timestamp'] - df['timestamp'].min()) * 1e-9

    # 3. 尝试计算欧拉角
    # df = calculate_euler_from_quat(df)
        
    # 简单过滤位置异常值
    if 'x' in df.columns:
        df = df[(np.abs(df['x']) < 5000) & (np.abs(df['y']) < 5000)]
        
    return df

def plot_and_save(uav_df, target_df, title_suffix, save_path):
    """绘制 4x3 布局的大图"""
    if uav_df.empty and target_df.empty:
        return

    fig = plt.figure(figsize=FIG_SIZE)
    # 布局: 4行 3列
    # Col 1: Pos X, Pos Y, Pos Z, 3D
    # Col 2: Roll, Pitch, Yaw, (Blank)
    # Col 3: Qx, Qy, Qz, Qw
    
    # --- 第1列：位置信息 ---
    ax_px = plt.subplot(4, 3, 1)
    if not uav_df.empty: ax_px.plot(uav_df['time_s'], uav_df['x'], 'r-', label='UAV', lw=1)
    if not target_df.empty: ax_px.plot(target_df['time_s'], target_df['x'], 'b-', label=f'Target', lw=1)
    ax_px.set_title(f'Position X ({title_suffix})')
    ax_px.grid(True, alpha=0.3); ax_px.legend(loc='upper right', fontsize='small')

    ax_py = plt.subplot(4, 3, 4)
    if not uav_df.empty: ax_py.plot(uav_df['time_s'], uav_df['y'], 'r-', lw=1)
    if not target_df.empty: ax_py.plot(target_df['time_s'], target_df['y'], 'b-', lw=1)
    ax_py.set_title('Position Y')
    ax_py.grid(True, alpha=0.3)

    ax_pz = plt.subplot(4, 3, 7)
    if not uav_df.empty: ax_pz.plot(uav_df['time_s'], uav_df['z'], 'r-', lw=1)
    if not target_df.empty: ax_pz.plot(target_df['time_s'], target_df['z'], 'b-', lw=1)
    ax_pz.set_title('Position Z')
    ax_pz.grid(True, alpha=0.3)

    # 3D 轨迹 (第1列第4行)
    ax_3d = plt.subplot(4, 3, 10, projection='3d')
    if not uav_df.empty: 
        ax_3d.plot(uav_df['x'], uav_df['y'], uav_df['z'], 'r-', label='UAV', lw=1)
        ax_3d.scatter(uav_df['x'].iloc[0], uav_df['y'].iloc[0], uav_df['z'].iloc[0], c='red', s=20)
    if not target_df.empty: 
        ax_3d.plot(target_df['x'], target_df['y'], target_df['z'], 'b-', label='Target', lw=1)
        ax_3d.scatter(target_df['x'].iloc[0], target_df['y'].iloc[0], target_df['z'].iloc[0], c='blue', s=20)
    ax_3d.set_title('3D Trajectory')

    # --- 第2列：欧拉角 (Roll, Pitch, Yaw) ---
    # 检查是否有欧拉角数据
    has_euler_uav = 'roll' in uav_df.columns
    has_euler_target = 'roll' in target_df.columns

    ax_roll = plt.subplot(4, 3, 2)
    if has_euler_uav: ax_roll.plot(uav_df['time_s'], uav_df['roll'], 'r-', lw=1)
    if has_euler_target: ax_roll.plot(target_df['time_s'], target_df['roll'], 'b-', lw=1)
    ax_roll.set_title('Roll (rad)')
    ax_roll.grid(True, alpha=0.3)

    ax_pitch = plt.subplot(4, 3, 5)
    if has_euler_uav: ax_pitch.plot(uav_df['time_s'], uav_df['pitch'], 'r-', lw=1)
    if has_euler_target: ax_pitch.plot(target_df['time_s'], target_df['pitch'], 'b-', lw=1)
    ax_pitch.set_title('Pitch (rad)')
    ax_pitch.grid(True, alpha=0.3)

    ax_yaw = plt.subplot(4, 3, 8)
    if has_euler_uav: ax_yaw.plot(uav_df['time_s'], uav_df['yaw'], 'r-', lw=1)
    if has_euler_target: ax_yaw.plot(target_df['time_s'], target_df['yaw'], 'b-', lw=1)
    ax_yaw.set_title('Yaw (rad)')
    ax_yaw.grid(True, alpha=0.3)
    
    # (第2列第4行留空或放 Legend)
    ax_blank = plt.subplot(4, 3, 11)
    ax_blank.axis('off')
    ax_blank.text(0.5, 0.5, f"Comparison: UAV vs Target\nMode: {title_suffix}", ha='center', va='center', fontsize=12)

    # --- 第3列：四元数 (Qx, Qy, Qz, Qw) ---
    has_quat_uav = 'qx' in uav_df.columns
    has_quat_target = 'qx' in target_df.columns

    ax_qx = plt.subplot(4, 3, 3)
    if has_quat_uav: ax_qx.plot(uav_df['time_s'], uav_df['qx'], 'r-', lw=1)
    if has_quat_target: ax_qx.plot(target_df['time_s'], target_df['qx'], 'b-', lw=1)
    ax_qx.set_title('Quat X')
    ax_qx.grid(True, alpha=0.3)

    ax_qy = plt.subplot(4, 3, 6)
    if has_quat_uav: ax_qy.plot(uav_df['time_s'], uav_df['qy'], 'r-', lw=1)
    if has_quat_target: ax_qy.plot(target_df['time_s'], target_df['qy'], 'b-', lw=1)
    ax_qy.set_title('Quat Y')
    ax_qy.grid(True, alpha=0.3)

    ax_qz = plt.subplot(4, 3, 9)
    if has_quat_uav: ax_qz.plot(uav_df['time_s'], uav_df['qz'], 'r-', lw=1)
    if has_quat_target: ax_qz.plot(target_df['time_s'], target_df['qz'], 'b-', lw=1)
    ax_qz.set_title('Quat Z')
    ax_qz.grid(True, alpha=0.3)

    ax_qw = plt.subplot(4, 3, 12)
    if has_quat_uav: ax_qw.plot(uav_df['time_s'], uav_df['qw'], 'r-', lw=1)
    if has_quat_target: ax_qw.plot(target_df['time_s'], target_df['qw'], 'b-', lw=1)
    ax_qw.set_title('Quat W')
    ax_qw.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(save_path, dpi=200, bbox_inches='tight')
    plt.close()
    print(f"已保存: {save_path}")

def batch_process_csv_plots(data_dir):
    print("\n>>> 开始处理 CSV 轨迹对比 (含姿态)...")
    uav_path = os.path.join(data_dir, "uav_states.csv")
    
    # 1. 读取 UAV 获取基准时间
    uav_df = read_trajectory_csv(uav_path, is_uav=True)
    if uav_df.empty:
        print("错误: UAV 数据为空，无法进行时间对齐")
        return None
    
    base_ts = uav_df['timestamp'].min()
    
    # 2. 定义任务列表
    tasks = [
        ("target_states_ori.csv", "Original", "trace_ori.png"),
        ("target_states_filtered.csv", "Filtered", "trace_filtered.png"),
        ("target_states_15hz.csv", "15Hz", "trace_15hz.png")
    ]
    
    # 3. 循环绘图
    for csv_name, title_suffix, png_name in tasks:
        target_path = os.path.join(data_dir, csv_name)
        target_df = read_trajectory_csv(target_path, is_uav=False, base_timestamp=base_ts)
        
        save_path = os.path.join(data_dir, png_name)
        plot_and_save(uav_df, target_df, title_suffix, save_path)
        
    return base_ts

# ================= 第二部分：二进制预测可视化 (保持不变) =================
# 这里直接复用你之前的代码，或者上面的部分代码。
# 为了保持脚本完整，我把解析 bin 的部分也粘在这里

def parse_binary_log(file_path):
    data_list = []
    n_vars = MODEL_CONFIG['n_vars']
    seq_len = MODEL_CONFIG['seq_len']
    pred_len = MODEL_CONFIG['pred_len']
    
    # Magic(8) + Ts(8) + Input + Output
    header_size = 16 
    input_bytes = seq_len * n_vars * 4
    output_bytes = pred_len * n_vars * 4
    block_size = header_size + input_bytes + output_bytes
    
    if not os.path.exists(file_path):
        return []

    print(f"正在解析二进制文件...")
    try:
        with open(file_path, "rb") as f:
            while True:
                chunk = f.read(block_size)
                if len(chunk) < block_size: break
                
                offset = 0
                magic, ts = struct.unpack('<QQ', chunk[0:16])
                offset += 16
                
                input_flat = struct.unpack(f'<{seq_len * n_vars}f', chunk[offset : offset + input_bytes])
                input_arr = np.array(input_flat).reshape(seq_len, n_vars)
                offset += input_bytes
                
                output_flat = struct.unpack(f'<{pred_len * n_vars}f', chunk[offset : offset + output_bytes])
                output_arr = np.array(output_flat).reshape(pred_len, n_vars)
                
                data_list.append({'timestamp': ts, 'input': input_arr, 'prediction': output_arr})
    except Exception as e:
        print(f"解析二进制出错: {e}")
            
    return data_list

def visualize_predictions_in_detail(bin_data, ground_truth_df, output_dir, base_ts_ns):
    if not bin_data: return
    pred_dir = os.path.join(output_dir, "pred")
    if os.path.exists(pred_dir): shutil.rmtree(pred_dir)
    os.makedirs(pred_dir)
    
    print(f"\n>>> 生成预测可视化 (保存至 {pred_dir})...")
    
    total_samples = len(bin_data)
    num_vis = 30
    step = max(total_samples // num_vis, 1)
    indices = range(0, total_samples, step)[:num_vis]
    
    feat_names = MODEL_CONFIG['feature_names']
    n_vars = MODEL_CONFIG['n_vars']
    
    gt_df = ground_truth_df.sort_values('timestamp')
    gt_timestamps = gt_df['timestamp'].values
    
    # 尝试映射 GT 列 (仅限前3维 Pos)
    csv_cols = ['x', 'y', 'z', 'roll', 'pitch', 'yaw']
    
    for idx in indices:
        sample = bin_data[idx]
        current_ts = sample['timestamp']
        hist_data = sample['input'] 
        pred_data = sample['prediction']
        
        # 寻找 GT
        start_idx = np.searchsorted(gt_timestamps, current_ts)
        valid_gt = False
        true_future = None
        
        if start_idx + MODEL_CONFIG['pred_len'] < len(gt_df):
            future_df_slice = gt_df.iloc[start_idx : start_idx + MODEL_CONFIG['pred_len']]
            true_future = np.full((MODEL_CONFIG['pred_len'], n_vars), np.nan)
            if all(c in gt_df.columns for c in csv_cols):
                true_future[:, 0:6] = future_df_slice[csv_cols].values
                valid_gt = True
        
        pdf_path = os.path.join(pred_dir, f'sample_{idx}_ts_{current_ts}.pdf')
        with PdfPages(pdf_path) as pdf:
            t_hist = np.linspace(-MODEL_CONFIG['seq_len']/FREQ_HZ, 0, MODEL_CONFIG['seq_len'])
            t_fut = np.linspace(0, MODEL_CONFIG['pred_len']/FREQ_HZ, MODEL_CONFIG['pred_len'])
            
            for f_i in range(n_vars):
                if f_i >= len(feat_names): break
                plt.figure(figsize=(10, 5))
                plt.plot(t_hist, hist_data[:, f_i], 'k-', label='History', alpha=0.7)
                conn_t = [t_hist[-1], t_fut[0]]
                conn_y = [hist_data[-1, f_i], pred_data[0, f_i]]
                plt.plot(conn_t, conn_y, 'g--', alpha=0.5)
                plt.plot(t_fut, pred_data[:, f_i], 'g.-', label='Prediction', lw=1.5)
                
                if valid_gt and not np.isnan(true_future[0, f_i]):
                    plt.plot(t_fut, true_future[:, f_i], 'r--', label='Ground Truth', alpha=0.6)
                
                plt.axvline(x=0, color='red', linestyle='-.', alpha=0.5)
                plt.title(f"{feat_names[f_i]} (Sample {idx})")
                plt.legend()
                plt.grid(True, alpha=0.3)
                plt.tight_layout()
                pdf.savefig()
                plt.close()

def main():
    base_ts_ns = batch_process_csv_plots(DATA_DIR)
    if base_ts_ns is None: return

    bin_path = os.path.join(DATA_DIR, "pred_result.bin")
    bin_data = parse_binary_log(bin_path)
    
    gt_csv_path = os.path.join(DATA_DIR, "target_states_filtered.csv")
    gt_df = read_trajectory_csv(gt_csv_path, is_uav=False) 
    
    if not gt_df.empty:
        visualize_predictions_in_detail(bin_data, gt_df, DATA_DIR, base_ts_ns)
    else:
        visualize_predictions_in_detail(bin_data, pd.DataFrame(), DATA_DIR, base_ts_ns)

    print("\n=== 完成 ===")

if __name__ == "__main__":
    main()
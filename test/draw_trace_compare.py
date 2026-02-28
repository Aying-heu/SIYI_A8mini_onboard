import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# 设置中文字体（避免中文乱码）
plt.rcParams['font.sans-serif'] = ['DejaVu Sans']  # 英文环境用这个
# plt.rcParams['font.sans-serif'] = ['SimHei']      # 中文Windows用这个
# plt.rcParams['axes.unicode_minus'] = False        # 解决负号显示问题
def read_trajectory_csv(file_path, is_uav=False, base_timestamp=None):
    """
    读取轨迹CSV文件
    :param file_path: 文件路径
    :param is_uav: 是否是无人机文件（区分列名）
    :param base_timestamp: 基准时间戳（统一用无人机的最小时间戳）
    :return: 处理后的DataFrame
    """
    # 读取CSV
    try:
        df = pd.read_csv(file_path)
        print(f"成功读取文件 {file_path}，原始数据行数：{len(df)}")
    except Exception as e:
        print(f"读取文件失败：{e}")
        return pd.DataFrame()  # 返回空DataFrame
    
    # 统一列名（适配无人机和目标的不同列名）
    if is_uav:
        rename_dict = {
            'position_x': 'x',
            'position_y': 'y',
            'position_z': 'z',
            'timestamp_ns': 'timestamp'
        }
    else:
        rename_dict = {
            'px': 'x',
            'py': 'y',
            'pz': 'z',
            'timestamp_ns': 'timestamp'
        }
    
    # 只重命名存在的列
    df.rename(columns={k: v for k, v in rename_dict.items() if k in df.columns}, inplace=True)
    
    # 检查必要列是否存在
    required_cols = ['x', 'y', 'z', 'timestamp']
    missing_cols = [col for col in required_cols if col not in df.columns]
    if missing_cols:
        print(f"缺少必要列：{missing_cols}")
        return pd.DataFrame()
    
    # ========== 核心修改1：移除原有的min_timestamp计算 ==========
    # 注释/删除这两行：
    # min_timestamp = df['timestamp'].min()
    # df['time_s'] = (df['timestamp'] - min_timestamp) * 1e-9
    
    # ========== 核心修改2：使用传入的基准时间计算相对时间 ==========
    if base_timestamp is not None:
        df['time_s'] = (df['timestamp'] - base_timestamp) * 1e-9
    else:
        # 备用：如果没传基准时间，用自身最小值（兼容旧逻辑）
        df['time_s'] = (df['timestamp'] - df['timestamp'].min()) * 1e-9
    
    # 宽松过滤（可选）
    print(f"过滤前X范围：{df['x'].min():.2f} ~ {df['x'].max():.2f}")
    print(f"过滤前Y范围：{df['y'].min():.2f} ~ {df['y'].max():.2f}")
    print(f"过滤前Z范围：{df['z'].min():.2f} ~ {df['z'].max():.2f}")
    df = df[(np.abs(df['x']) < 1000) & (np.abs(df['y']) < 1000) & (np.abs(df['z']) < 1000)]
    print(f"过滤后数据行数：{len(df)}")
    
    return df

def plot_trajectory_comparison(uav_df, target_df):
    """
    可视化对比无人机和目标的轨迹（新增空数据检查）
    """
    # ========== 修复3：空数据检查 ==========
    if uav_df.empty and target_df.empty:
        print("错误：无人机和目标数据都为空！")
        return
    if uav_df.empty:
        print("警告：无人机数据为空，仅显示目标轨迹")
    if target_df.empty:
        print("警告：目标数据为空，仅显示无人机轨迹")
    
    # 创建画布：分为X/Y/Z三轴对比 + 3D轨迹
    fig = plt.figure(figsize=(15, 12))
    
    # 1. X轴位置对比
    ax1 = plt.subplot(4, 1, 1)
    if not uav_df.empty:
        ax1.plot(uav_df['time_s'], uav_df['x'], 'r-', label='UAV X', linewidth=1.5)
    if not target_df.empty:
        ax1.plot(target_df['time_s'], target_df['x'], 'b-', label='Target X', linewidth=1.5)
    ax1.set_title('X Position Comparison', fontsize=12)
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('X (m)')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    # 2. Y轴位置对比
    ax2 = plt.subplot(4, 1, 2)
    if not uav_df.empty:
        ax2.plot(uav_df['time_s'], uav_df['y'], 'r-', label='UAV Y', linewidth=1.5)
    if not target_df.empty:
        ax2.plot(target_df['time_s'], target_df['y'], 'b-', label='Target Y', linewidth=1.5)
    ax2.set_title('Y Position Comparison', fontsize=12)
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Y (m)')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    # 3. Z轴位置对比
    ax3 = plt.subplot(4, 1, 3)
    if not uav_df.empty:
        ax3.plot(uav_df['time_s'], uav_df['z'], 'r-', label='UAV Z', linewidth=1.5)
    if not target_df.empty:
        ax3.plot(target_df['time_s'], target_df['z'], 'b-', label='Target Z', linewidth=1.5)
    ax3.set_title('Z Position Comparison', fontsize=12)
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Z (m)')
    ax3.legend()
    ax3.grid(True, alpha=0.3)
    
    # 4. 3D轨迹对比
    ax4 = plt.subplot(4, 1, 4, projection='3d')
    # 绘制无人机3D轨迹
    if not uav_df.empty:
        ax4.plot(uav_df['x'], uav_df['y'], uav_df['z'], 'r-', label='UAV Trajectory', linewidth=2)
        # 仅当有数据时标记起点
        ax4.scatter(uav_df['x'].iloc[0], uav_df['y'].iloc[0], uav_df['z'].iloc[0], 
                    c='red', s=50, label='UAV Start', zorder=5)
    # 绘制目标3D轨迹
    if not target_df.empty:
        ax4.plot(target_df['x'], target_df['y'], target_df['z'], 'b-', label='Target Trajectory', linewidth=2)
        # 仅当有数据时标记起点
        ax4.scatter(target_df['x'].iloc[0], target_df['y'].iloc[0], target_df['z'].iloc[0], 
                    c='blue', s=50, label='Target Start', zorder=5)
    ax4.set_title('3D Trajectory Comparison', fontsize=12)
    ax4.set_xlabel('X (m)')
    ax4.set_ylabel('Y (m)')
    ax4.set_zlabel('Z (m)')
    ax4.legend()
    ax4.grid(True, alpha=0.3)
    
    # 调整布局
    plt.tight_layout()
    
    # 保存图片（高清）
    plt.savefig('uav_target_trajectory_comparison.png', dpi=300, bbox_inches='tight')
    plt.show()

    # 额外：计算位置误差并输出（新增空数据检查）
    print("\n=== 轨迹误差统计 ===")
    if uav_df.empty or target_df.empty:
        print("数据为空，无法计算误差")
        return
    
    # 对齐时间（取较短的时间序列）
    min_time = min(uav_df['time_s'].max(), target_df['time_s'].max())
    uav_aligned = uav_df[uav_df['time_s'] <= min_time]
    target_aligned = target_df[target_df['time_s'] <= min_time]
    
    if uav_aligned.empty or target_aligned.empty:
        print("时间对齐后数据为空，无法计算误差")
        return
    
    # 插值对齐数据点
    target_interp = target_aligned.set_index('time_s').reindex(uav_aligned['time_s']).interpolate()
    target_interp['time_s'] = uav_aligned['time_s'].values
    
    # 过滤插值后的空值
    target_interp = target_interp.dropna()
    if target_interp.empty:
        print("插值后无有效数据，无法计算误差")
        return
    
    # 计算各轴误差
    error_x = uav_aligned['x'].values[:len(target_interp)] - target_interp['x'].values
    error_y = uav_aligned['y'].values[:len(target_interp)] - target_interp['y'].values
    error_z = uav_aligned['z'].values[:len(target_interp)] - target_interp['z'].values
    total_error = np.sqrt(error_x**2 + error_y**2 + error_z**2)
    
    print(f"X轴平均误差: {np.mean(np.abs(error_x)):.4f} m")
    print(f"Y轴平均误差: {np.mean(np.abs(error_y)):.4f} m")
    print(f"Z轴平均误差: {np.mean(np.abs(error_z)):.4f} m")
    print(f"整体平均误差: {np.mean(total_error):.4f} m")
    print(f"最大误差: {np.max(total_error):.4f} m")

if __name__ == "__main__":
    # === 替换为你的CSV文件路径 ===
    UAV_CSV_PATH = "/home/bsa/A_vision_relate/data_20260128_171950/uav_states.csv"     
    TARGET_CSV_PATH = "/home/bsa/A_vision_relate/data_20260128_171950/target_states_15hz.csv"  
    
    # 读取无人机数据
    print("正在读取无人机轨迹数据...")
    uav_df = read_trajectory_csv(UAV_CSV_PATH, is_uav=True)
    
    # ========== 核心修改3：获取无人机的最小时间戳作为基准 ==========
    base_timestamp = None
    if not uav_df.empty:
        base_timestamp = uav_df['timestamp'].min()
        print(f"无人机基准时间戳：{base_timestamp}")
    else:
        print("无人机数据为空，无法获取基准时间！")
        exit(1)
    
    # ========== 核心修改4：读取目标数据时传入无人机基准时间 ==========
    print("正在读取目标轨迹数据...")
    target_df = read_trajectory_csv(TARGET_CSV_PATH, is_uav=False, base_timestamp=base_timestamp)
    
    # 可视化对比
    print("正在生成可视化图表...")
    plot_trajectory_comparison(uav_df, target_df)
    
    print("完成！图表已保存为 uav_target_trajectory_comparison.png")
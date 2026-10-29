import matplotlib.pyplot as plt
import matplotlib.patches as patches
import math

plt.rcParams['font.sans-serif'] = ['SimHei']
plt.rcParams['axes.unicode_minus'] = False

NO_FLY_ZONES = [
    {'name': 'Airport',      'lat': 47.65, 'lon': -122.14, 'radius_m': 2000},
    {'name': 'MilitaryBase', 'lat': 47.60, 'lon': -122.20, 'radius_m': 1500},
    {'name': 'School',       'lat': 47.62, 'lon': -122.10, 'radius_m': 500},
]

def plot_zones_and_drone(drone_lat, drone_lon):
    fig, ax = plt.subplots(figsize=(10, 8))
    
    for zone in NO_FLY_ZONES:
        x = (zone['lon'] - (-122.20)) * 111000 * math.cos(math.radians(47.6))
        y = (zone['lat'] - 47.55) * 111000
        radius = zone['radius_m']
        
        circle = patches.Circle((x, y), radius, color='red', alpha=0.3, label=zone['name'])
        ax.add_patch(circle)
        ax.text(x, y, zone['name'], color='black', ha='center', va='center', fontweight='bold')

    drone_x = (drone_lon - (-122.20)) * 111000 * math.cos(math.radians(47.6))
    drone_y = (drone_lat - 47.55) * 111000
    ax.plot(drone_x, drone_y, marker='^', color='green', markersize=15, label='Drone Position')
    
    ax.set_title('No-Fly Zone Visualization (2D)', fontsize=16)
    ax.set_xlabel('X Coordinate (meters)', fontsize=12)
    ax.set_ylabel('Y Coordinate (meters)', fontsize=12)
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.axis('equal')
    
    handles, labels = ax.get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    ax.legend(by_label.values(), by_label.keys(), loc='upper right')
    
    plt.savefig('visualization_result.png', dpi=300, bbox_inches='tight')
    print("可视化结果已保存为 visualization_result.png")
    
    plt.show()

if __name__ == '__main__':
    print("正在生成禁飞区可视化图...")
    demo_drone_lat = 47.64
    demo_drone_lon = -122.15
    plot_zones_and_drone(demo_drone_lat, demo_drone_lon)
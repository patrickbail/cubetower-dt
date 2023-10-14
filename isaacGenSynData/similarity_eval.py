import os
import cv2
import json
#import pptk
import skimage
import numpy as np
import open3d as o3d
import point_cloud_utils as pcu
#import scipy.spatial as spatial
from PIL import Image
from sklearn.cluster import DBSCAN
from tqdm import tqdm
from json import JSONEncoder
from queue import PriorityQueue
from matplotlib import pyplot as plt
from matplotlib import cm
import matplotlib.colors as mcol
from image_similarity_measures.quality_metrics import rmse, ssim, psnr, sre

def load_image(file, no_alpha=True):
    image = np.asarray(Image.open(file))
    if no_alpha and len(image) > 2 and image.shape[2] == 4:
        image = image[:,:,:3]
    return image[:,:,::-1].copy()

class NumpyArrayEncoder(JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        return JSONEncoder.default(self, obj)

def export_to_json(json_obj, file_name, dir_name):
    dir_name = os.path.dirname(dir_name)
    os.makedirs(dir_name, exist_ok=True)

    file_name = file_name + ".json"
    file = os.path.join(dir_name, file_name)
    with open(file, 'w', encoding='utf-8') as f:
        json.dump(json_obj, f, cls=NumpyArrayEncoder)

def measure_img_similarity(name, limit, blur=False, noise=False, toJson=False, offset=0, max_p=255, cut_off=None):
    # Priority queues for listing top k results
    rmse_q = PriorityQueue()
    psnr_q = PriorityQueue()
    sre_q = PriorityQueue()
    ssim_q = PriorityQueue()

    rmse_vals = []
    psnr_vals = []
    sre_vals = []
    ssim_vals = []

    additional_data = []

    # Distance data from LiDAR scan neccessary
    lidar_eval_file = './isaacGenSynData/results/pcd/Pcd_RTX_eval_results.json'
    with open(lidar_eval_file, 'r', encoding='utf-8') as jsonf:
        lidar_eval = json.load(jsonf)

    distances = np.asarray([dist["Closest distance"] for dist in lidar_eval["Additional data"]])
    new_size = limit-offset
    interpolated_distances = np.interp(np.linspace(0, distances.shape[0], new_size), np.arange(distances.shape[0]), distances)

    for i in tqdm(range(offset, limit), desc="Processing"):
        # Cut out data
        if cut_off:
            if cut_off[0] <= i <= cut_off[1]: 
                continue 
        real_img = f'./isaacGenSynData/final_cube_scan/raw_rgb/{i+1}_img.png'
        syn_img = f'./isaacGenSynData/_out_no_interpolation_image/{name}_step_{i}.png'
        try:
            RI = load_image(real_img)
            SI = load_image(syn_img)
        except:
            #only until 2232
            print(i)
            continue
        if blur:
            SI = cv2.GaussianBlur(SI,(5,5),0)

        # Calculate similarity metrics
        rmse_r = rmse(org_img=RI, pred_img=SI, max_p=max_p)
        psnr_r = psnr(org_img=RI, pred_img=SI, max_p=max_p)
        sre_r = sre(org_img=RI, pred_img=SI)
        ssim_r = ssim(org_img=RI, pred_img=SI, max_p=max_p)
        
        rmse_q.put((rmse_r, i))
        rmse_vals.append(rmse_r.astype(float))
        psnr_q.put((-psnr_r, i))
        psnr_vals.append(psnr_r.astype(float))
        sre_q.put((-sre_r, i))
        sre_vals.append(sre_r.astype(float))
        ssim_q.put((-ssim_r, i))
        ssim_vals.append(ssim_r.astype(float))

        additional_data.append({"Index": i, "Closest distance": interpolated_distances[i-offset]})
    
    rmse_vals = np.asarray(rmse_vals)
    psnr_vals = np.asarray(psnr_vals)
    sre_vals = np.asarray(sre_vals)
    ssim_vals = np.asarray(ssim_vals)

    mean_rmse = np.mean(rmse_vals)
    mean_psnr = np.mean(psnr_vals)
    mean_sre = np.mean(sre_vals)
    mean_ssim = np.mean(ssim_vals)

    # Calculate which synthetic images had the best value
    best_rmse_index = np.argmin(rmse_vals)
    best_psnr_index = np.argmax(psnr_vals)
    best_sre_index = np.argmax(sre_vals)
    best_ssim_index = np.argmax(ssim_vals)
    best_rmse = {"Value": rmse_vals[best_rmse_index], "Index": additional_data[best_rmse_index]["Index"], 
                    "Closest distance": additional_data[best_rmse_index]["Closest distance"]}
    best_psnr = {"Value": psnr_vals[best_psnr_index], "Index": additional_data[best_psnr_index]["Index"], 
                    "Closest distance": additional_data[best_psnr_index]["Closest distance"]}
    best_sre = {"Value": sre_vals[best_sre_index], "Index": additional_data[best_sre_index]["Index"], 
                    "Closest distance": additional_data[best_sre_index]["Closest distance"]}
    best_ssim = {"Value": ssim_vals[best_ssim_index], "Index": additional_data[best_ssim_index]["Index"], 
                    "Closest distance": additional_data[best_ssim_index]["Closest distance"]}

    vals_dict = {"blur": blur, "noise": noise, "Additional data": additional_data,
                 "RMSE": {"Mean RMSE": mean_rmse, "Best value": best_rmse, "values": rmse_vals},
                 "PSNR": {"Mean PSNR": mean_psnr, "Best value": best_psnr, "values": psnr_vals},
                 "SRE": {"Mean SRE": mean_sre, "Best value": best_sre, "values": sre_vals},
                 "SSIM": {"Mean SSIM": mean_ssim, "Best value": best_ssim, "values": ssim_vals}}

    if toJson:
        export_to_json(vals_dict, "No_Interpolation_Image_eval_results", "./isaacGenSynData/results/image/")

    print(f"Mean RMSE = {mean_rmse}")
    print(f"Mean PSNR = {mean_psnr}")
    print(f"Mean SRE = {mean_sre}")
    print(f"Mean SSIM = {mean_ssim}")
    return (rmse_q, psnr_q, sre_q, ssim_q)

# cut from 1606 1960
def test_image_similarity():
    num_imgs = 2235 #2238 #1736
    rmse_q, psnr_q, sre_q, ssim_q = measure_img_similarity("left", num_imgs, blur=False, noise=True, toJson=True, offset=3, cut_off=[1606, 1960])

    for j in range(10):
        print(f"--Top {j + 1}--")
        print(f"RMSE: {rmse_q.get()}")
        print(f"PSNR: {psnr_q.get()}")
        print(f"SRE: {sre_q.get()}" )
        print(f"SSIM: {ssim_q.get()}")

def closest_furthest_point(point, points):
    # Determine the closest and furthest point from a set of points to a position
    dist_2 = np.linalg.norm(points - point, axis=1)
    return np.argmin(dist_2), np.min(dist_2), np.argmax(dist_2), np.max(dist_2)

def detect_clusters(point_cloud):
    # eps and min_samples influence on cluster detection
    #dbscan = DBSCAN(eps=0.5, min_samples=5) # Worked good for first LiDAR scan
    dbscan = DBSCAN(eps=0.2, min_samples=8)
    cluster_labels = dbscan.fit_predict(point_cloud)

    unique_labels = np.unique(cluster_labels)
    #print("Total amount of clusters: ", unique_labels)
    cluster_centroids = []

    for label in unique_labels:
        if label != -1:
            # Calculate centroids of clusters
            cluster = point_cloud[cluster_labels == label]
            centroid = np.mean(cluster, axis=0)
            cluster_centroids.append(centroid)

    # Select cluster closest to origin
    distances_to_origin = [np.linalg.norm(centroid - np.zeros(3)) for centroid in cluster_centroids]
    closest_cluster_idx = np.argmin(distances_to_origin)
    return point_cloud[cluster_labels == unique_labels[closest_cluster_idx]]

def mask_pc(*pcd, x_bound=[-0.1, 3], y_bound=[-3.2, 0], rotate=None):
    results = []
    for pc in pcd:
        if rotate:
            # Convert the rotation angle from degrees to radians
            angle_rad = np.deg2rad(rotate)
            # Define the rotation matrix
            rotation_matrix = np.array([
                [np.cos(angle_rad), -np.sin(angle_rad), 0],
                [np.sin(angle_rad), np.cos(angle_rad), 0],
                [0, 0, 1]
            ])
            # Apply the rotation to the point cloud
            pc = np.dot(pc, rotation_matrix.T)

        x, y, z = pc[:, 0], pc[:, 1], pc[:, 2]
        mask = (x > x_bound[0]) & (x < x_bound[1]) & (y > y_bound[0]) & (y < y_bound[1]) & (z > -0.9) & (z < -0.08)
        pc = pc[mask]

        pc_data = o3d.geometry.PointCloud()
        pc_data.points = o3d.utility.Vector3dVector(pc)
        results.append(pc_data)

    if len(results) == 1:
        return results[0]
    else:
        return tuple(results)
    
def cleanse_pc_data(r_points, s_points, outlier_epsilon=0.4, isPhysX=False):
    # Cleanse point cloud data from unwanted information
    if isPhysX:
        # Mask out initial known irrlevant points
        real_pcd = mask_pc(r_points, x_bound=[-0.74, 0.74], y_bound=[-2.7, -0.33], rotate=-45)
        syn_pcd = mask_pc(s_points, x_bound=[-10, 10], y_bound=[-10, 0], rotate=-45)
        r_points = np.asarray(real_pcd.points)
        s_points = np.asarray(syn_pcd.points)

        # Check for outliers
        voxel_down_real_pcd = real_pcd.voxel_down_sample(voxel_size=0.02)
        voxel_down_syn_pcd = syn_pcd.voxel_down_sample(voxel_size=0.02)
        # STD_ratio influence on outliers
        _, ind = voxel_down_real_pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=3)
        real_pcd = voxel_down_real_pcd.select_by_index(ind)
        r_points = np.asarray(real_pcd.points)

        # Check if objects which were not outliers exist and remove them
        _, closest_dist, _, outlier_obj_dist = closest_furthest_point(np.zeros(3), r_points)

        # Remove outlier objects if necessary
        removedOutlierObj = False
        outlier_dist = outlier_obj_dist - closest_dist
        if outlier_dist > outlier_epsilon:
            r_points = detect_clusters(r_points)
            real_pcd = o3d.geometry.PointCloud()
            real_pcd.points = o3d.utility.Vector3dVector(r_points)
            r_points = np.asarray(real_pcd.points)
            removedOutlierObj = True

        return real_pcd, syn_pcd, removedOutlierObj, closest_dist, outlier_dist

    # Mask out initial known irrlevant points
    real_pcd, syn_pcd = mask_pc(r_points, s_points, x_bound=[-0.74, 0.74], y_bound=[-2.7, -0.33], rotate=-45)
    r_points = np.asarray(real_pcd.points)
    s_points = np.asarray(syn_pcd.points)

    # Check for outliers
    voxel_down_real_pcd = real_pcd.voxel_down_sample(voxel_size=0.02)
    voxel_down_syn_pcd = syn_pcd.voxel_down_sample(voxel_size=0.02)
    # STD_ratio influence on outliers
    _, ind = voxel_down_real_pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=3)
    real_pcd = voxel_down_real_pcd.select_by_index(ind)
    r_points = np.asarray(real_pcd.points)

    _, ind = voxel_down_syn_pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=3.5)
    syn_pcd = voxel_down_syn_pcd.select_by_index(ind)
    s_points = np.asarray(syn_pcd.points)

    # Check if objects which were not outliers exist and remove them
    _, closest_dist, _, outlier_obj_dist = closest_furthest_point(np.zeros(3), r_points)

    # Remove outlier object if necessary
    removedOutlierObj = False
    outlier_dist = outlier_obj_dist - closest_dist
    if outlier_dist > outlier_epsilon:
        r_points = detect_clusters(r_points)
        real_pcd = o3d.geometry.PointCloud()
        real_pcd.points = o3d.utility.Vector3dVector(r_points)
        r_points = np.asarray(real_pcd.points)

        s_points = detect_clusters(s_points)
        syn_pcd = o3d.geometry.PointCloud()
        syn_pcd.points = o3d.utility.Vector3dVector(s_points)
        s_points = np.asarray(syn_pcd.points)

        removedOutlierObj = True
    
    return real_pcd, syn_pcd, removedOutlierObj, closest_dist, outlier_dist

def measure_pcd_similarity(limit, noise=False, toJson=False, offset=0, threshold=0.6, cut_out=None):
    # Priority queues for listing top k results
    chamfer_q = PriorityQueue()
    hausdorff_q = PriorityQueue()
    rmse_q = PriorityQueue()

    chamfer_vals = []
    hausdorff_vals = []
    rmse_vals = []

    additional_data = []

    for i in tqdm(range(offset, limit), desc="Processing"):
        if cut_out:
            if i in cut_out:
                continue
        real_pcd_file = f'./isaacGenSynData/final_cube_scan/raw_pcd/{i+1}_pcd.ply'
        #syn_pcd_file = f'./isaacGenSynData/_out_no_interpolation_pcd/synthetic_{i}_pcd.ply' #RTX
        syn_pcd_file = f'./isaacGenSynData/_out_no_interpolation_physx_pcd/synthetic_{i}_physx_pcd.ply' #PhysX
        
        real_pcd = o3d.io.read_point_cloud(real_pcd_file)
        syn_pcd = o3d.io.read_point_cloud(syn_pcd_file)
        r_points = np.asarray(real_pcd.points)
        s_points = np.asarray(syn_pcd.points)

        real_pcd, syn_pcd, removedOutlierObj, closest_dist, outlier_dist = cleanse_pc_data(r_points, s_points, isPhysX=True)
        r_points = np.asarray(real_pcd.points)
        s_points = np.asarray(syn_pcd.points)

        # Calculate point density in a 5cm radius with KDTrees
        #radius = 0.05 # 5cm
        #r_kdtree = spatial.KDTree(np.array(r_points))
        #s_kdtree = spatial.KDTree(np.array(s_points))
        #r_neighbors = r_kdtree.query_ball_tree(r_kdtree, radius)
        #s_neighbors = s_kdtree.query_ball_tree(s_kdtree, radius)
        #r_frequency = np.array([len(i) for i in r_neighbors])
        #s_frequency = np.array([len(i) for i in s_neighbors])
        #r_density = r_frequency/radius**2
        #s_density = s_frequency/radius**2
        #r_density_mean = np.mean(r_density)
        #s_density_mean = np.mean(s_density)

        if noise:
            mu, sigma = 0, 0.001 # 1mm
            gaussian = np.random.normal(mu, sigma, s_points.shape)
            s_points += gaussian
            syn_pcd = o3d.geometry.PointCloud()
            syn_pcd.points = o3d.utility.Vector3dVector(s_points)
            s_points = np.asarray(syn_pcd.points)

        # Calculate metrics
        chamfer_r = pcu.chamfer_distance(r_points, s_points)
        hausdorff_r = pcu.hausdorff_distance(r_points, s_points)
        rmse_r = o3d.pipelines.registration.evaluate_registration(real_pcd, syn_pcd, threshold, np.identity(4)).inlier_rmse
        
        chamfer_q.put((chamfer_r, i))
        chamfer_vals.append(chamfer_r)
        hausdorff_q.put((hausdorff_r, i))
        hausdorff_vals.append(hausdorff_r)
        rmse_q.put((rmse_r, i))
        rmse_vals.append(rmse_r)

        additional_data.append({"Index": i, "Closest distance": closest_dist, 
                                "Removed Outlier Object": removedOutlierObj, "Outlier Object distance": outlier_dist,
                                "Real amount of points": r_points.shape[0], "Synthetic amount of points": s_points.shape[0],})
                                #"Real point density": r_density_mean, "Synthetic point density": s_density_mean})
    
    chamfer_vals = np.asarray(chamfer_vals)
    hausdorff_vals = np.asarray(hausdorff_vals)
    rmse_vals = np.asarray(rmse_vals)

    # Calculate mean values for metrics
    mean_chamfer = np.mean(chamfer_vals)
    mean_hausdorff = np.mean(hausdorff_vals)
    mean_rmse = np.mean(rmse_vals)

    # Calculate the mean number of points in the point cloud
    mean_r_points = np.mean([item['Real amount of points'] for item in additional_data])
    mean_s_points = np.mean([item['Synthetic amount of points'] for item in additional_data])

    # Calculate the mean of point density
    #mean_r_density = np.mean(item['Real point density'][0] for item in additional_data)
    #mean_s_density = np.mean(item['Synthetic point density'][0] for item in additional_data)

    # Calculate which point clouds had the best value at what distance
    best_chamfer_index = np.argmin(chamfer_vals)
    best_hausdorff_index = np.argmin(hausdorff_vals)
    best_rmse_index = np.argmin(rmse_vals)
    best_chamfer = {"Value": chamfer_vals[best_chamfer_index], 
                    "Index": additional_data[best_chamfer_index]["Index"], 
                    "Closest distance": additional_data[best_chamfer_index]["Closest distance"]}
    best_hausdorff = {"Value": hausdorff_vals[best_hausdorff_index], 
                    "Index": additional_data[best_hausdorff_index]["Index"], 
                    "Closest distance": additional_data[best_hausdorff_index]["Closest distance"]}
    best_rmse = {"Value": rmse_vals[best_rmse_index], 
                    "Index": additional_data[best_rmse_index]["Index"], 
                    "Closest distance": additional_data[best_rmse_index]["Closest distance"]}

    vals_dict = {"noise": noise, "Mean Real point amount": mean_r_points, "Mean Synthetic point amount": mean_s_points, 
                 #"Mean Real point density": mean_r_density, "Mean Synthetic point density": mean_s_density, 
                 "Additional data": additional_data, 
                 "Chamfer": {"Mean Chamfer": mean_chamfer, "Best value": best_chamfer, "values": chamfer_vals},
                 "Hausdorff": {"Mean Hausdorff": mean_hausdorff, "Best value": best_hausdorff, "values": hausdorff_vals},
                 "RMSE": {"Mean RMSE": mean_rmse, "Best value": best_rmse, "values": rmse_vals}}

    if toJson:
        #export_to_json(vals_dict, "No_interpolation_Pcd_RTX_eval_results", "./isaacGenSynData/results/pcd/")
        export_to_json(vals_dict, "No_interpolation_Pcd_PhysX_eval_results", "./isaacGenSynData/results/pcd/")

    print(f"Mean Chamfer = {mean_chamfer}")
    print(f"Mean Hausdorff = {mean_hausdorff}")
    print(f"Mean RMSE = {mean_rmse}")
    return (chamfer_q, hausdorff_q, rmse_q)

def test_pointcloud_similarity():
    num_pcds = 748 #1160 #1240 #1217
    cut_out = [217, 539, 541, 142, 439, 441, 679, 466, 684, 468, 38, 686, 470, 477, 688, 132, 471, 472, 89, 3, 131]
    chamfer_q, hausdorff_q, rmse_q = measure_pcd_similarity(num_pcds, noise=False, toJson=True, offset=3, cut_out=cut_out)

    for j in range(10):
        print(f"--Top {j + 1}--")
        print(f"Chamfer: {chamfer_q.get()}")
        print(f"Hausdorff: {hausdorff_q.get()}")
        print(f"RMSE: {rmse_q.get()}")

def plot_all_data(file, title, isImage, *metrics):

    with open(file, 'r', encoding='utf-8') as jsonf:
        vals = json.load(jsonf)

    metric_vals = []
    for metric in metrics:
        metric_vals.append([metric, np.asarray(vals[metric]["values"])])

    closest_distance = np.asarray([item["Closest distance"] for item in vals["Additional data"]])

    x = np.arange(0, metric_vals[0][1].shape[0])

    cmap=cm.get_cmap('viridis')
    cnorm = mcol.Normalize(vmin=np.min(closest_distance),vmax=np.max(closest_distance))
    cpick = cm.ScalarMappable(norm=cnorm,cmap=cmap)
    cpick.set_array([])

    fig, axes = plt.subplots(1, len(metric_vals), figsize=(15, 6))

    for i in range(1, len(x)):
        for j, metric_val in enumerate(metric_vals):
            axes[j].fill_between([x[i - 1], x[i]], 0, metric_val[1][i], color=cpick.to_rgba(closest_distance[i]), interpolate=True, alpha=0.5)

    for i, metric_val in enumerate(metric_vals):
        axes[i].set_xlabel('Time')
        if metric_val[0] == "Hausdorff" or metric_val[0] == "Chamfer":
            axes[i].set_ylabel(f'{metric_val[0]} distance (m)')
        elif metric_val[0] == "PSNR" or metric_val[0] == "SRE":
            axes[i].set_ylabel(f'{metric_val[0]} (dB)')
        elif metric_val[0] == "RMSE" and not isImage:
            axes[i].set_ylabel(f'{metric_val[0]} (m)')
        else:
            axes[i].set_ylabel(f'{metric_val[0]}')
        axes[i].set_title(metric_val[0])

    for ax in axes:
        ax.grid(axis='y')
        ax.set_axisbelow(True)
        if isImage:
            ax.set_xticks(np.arange(min(x), max(x)+1, 200.0))
        else:
            ax.set_xticks(np.arange(min(x), max(x)+1, 100.0))

    plt.colorbar(cpick,label="Distance to cube tower (m)")

    fig.suptitle(title, fontsize="x-large")
    # Show the plot
    plt.tight_layout()
    fig.savefig(f'./isaacGenSynData/results/{title.replace(" ", "_")}.png')
    plt.show()

def plot_data_timeline(eval_file, isImage, metric, title):
    with open(eval_file, 'r', encoding='utf-8') as jsonf:
        val_obj = json.load(jsonf)

    vals = np.asarray(val_obj[metric]["values"])

    closest_distance = np.asarray([item["Closest distance"] for item in val_obj["Additional data"]])

    x = np.arange(0, vals.shape[0])

    cmap=cm.get_cmap('viridis')
    cnorm = mcol.Normalize(vmin=np.min(closest_distance),vmax=np.max(closest_distance))
    cpick = cm.ScalarMappable(norm=cnorm,cmap=cmap)
    cpick.set_array([])

    for i in range(1, len(x)):
        plt.fill_between([x[i - 1], x[i]], 0, vals[i], color=cpick.to_rgba(closest_distance[i]), interpolate=True, alpha=0.5, zorder=2.0)
    plt.xlabel('Time')
    if metric == "Hausdorff" or metric == "Chamfer":
        plt.ylabel(f'{metric} distance (m)')
    elif metric == "PSNR" or metric == "SRE":
        plt.ylabel(f'{metric} (dB)')
    elif metric == "RMSE" and not isImage:
        plt.ylabel(f'{metric} (m)')
    else:
        plt.ylabel(f'{metric}')
    plt.title(f'{title}: {metric}')
    if isImage:
        plt.xticks(np.arange(min(x), max(x)+1, 200.0))
    else:
        plt.xticks(np.arange(min(x), max(x)+1, 100.0))


    plt.grid(axis='y',zorder=-1.0)

    plt.colorbar(cpick,label="Distance to cube tower (m)")

    # Show the plot
    plt.tight_layout()
    if isImage:
        plt.savefig(f'./isaacGenSynData/results/image/figures_no_interpolation/{title.replace(" ", "_")}_{metric}.png', dpi=100)
    else:
        plt.savefig(f'./isaacGenSynData/results/pcd/figures_no_interpolation/{title.replace(" ", "_")}_{metric}.png', dpi=100)
    plt.show()

def plot_box_plot(file, title, *metrics):
    with open(file, 'r', encoding='utf-8') as jsonf:
        lidar_eval = json.load(jsonf)

    values = []
    for metric in metrics:
        values.append(np.asarray(lidar_eval[metric]["values"]))

    fig, ax = plt.subplots(figsize=(10, 6))
    # Create a boxplot
    bp = ax.boxplot(values, showfliers=False)

    # Add labels to the x-axis
    ax.set_xticklabels(metrics)
    pos = np.arange(len(metrics)) + 1
    medians = [med.get_ydata()[0] for med in bp['medians']]
    print(medians)
    upper_labels = [str(round(s, 2)) for s in medians]

    for i in range(len(metrics)):
        ax.text(pos[i]+0.3, .95, "Median: " + upper_labels[i],
                transform=ax.get_xaxis_transform(),
                horizontalalignment='center', size=11, color='royalblue')

    # Add a title and labels
    ax.set_title(f'{title}: Boxplot of Metrics')
    ax.set_xlabel('Metrics')
    ax.set_ylabel('Values')
    ax.yaxis.grid(True, linestyle='-', which='major', color='lightgrey',
                alpha=0.5)

    # Show the plot
    if len(metrics) > 1:
        fig.savefig(f'./isaacGenSynData/results/{title.replace(" ", "_")}.png')
    else:
        fig.savefig(f'./isaacGenSynData/results/{title.replace(" ", "_")}_{metrics[0]}.png')
    plt.show()

def plot_image_contours(index):
    real_img = f'./isaacGenSynData/real_data/final_cube_scan/raw_rgb/{index+1}_img.png'
    syn_img = f'./isaacGenSynData/synthetic_data/_out_image/left_step_{index}.png'

    RI = load_image(real_img)
    SI = load_image(syn_img)

    # Convert images to grayscale
    RIG = cv2.cvtColor(RI, cv2.COLOR_BGR2GRAY)
    SIG = cv2.cvtColor(SI, cv2.COLOR_BGR2GRAY)

    # Get grayscale difference of images
    #score, diff = skimage.metrics.structural_similarity(first, second, full=True, channel_axis=2)
    _, diff = skimage.metrics.structural_similarity(RIG, SIG, full=True)

    # Threshold and extract contours from difference image
    diff = (diff * 255).astype(np.uint8)
    _, thresh = cv2.threshold(diff, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)
    contours = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = contours[0] if len(contours) == 2 else contours[1]

    # Highlight differences
    mask = np.zeros(RI.shape, dtype='uint8')

    # Draw contours 
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 100:
            x, y, w, h = cv2.boundingRect(cnt)
            cv2.rectangle(RI, (x, y), (x + w, y + h), (36,255,12), 2)
            cv2.rectangle(SI, (x, y), (x + w, y + h), (36,255,12), 2)
            cv2.drawContours(mask, [cnt], 0, (0,255,0), -1)

    fig, axs = plt.subplots(2, 2)

    axs[0, 0].imshow(RI)
    axs[0, 1].imshow(SI)
    axs[1, 0].imshow(diff, cmap='gray')
    axs[1, 1].imshow(mask)

    plt.show()

    cv2.imwrite(f'./isaacGenSynData/results/image/real_{index+1}_contour.png', RI) 
    cv2.imwrite(f'./isaacGenSynData/results/image/syn_{index}_contour.png', SI) 
    cv2.imwrite(f'./isaacGenSynData/results/image/diff_{index}.png', diff) 
    cv2.imwrite(f'./isaacGenSynData/results/image/mask_{index}.png', mask)
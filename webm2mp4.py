from moviepy.editor import VideoFileClip
import os

def convert_webm_to_mp4(input_file, output_file):
    # 加载 WEBM 视频文件
    clip = VideoFileClip(input_file)
    # 将视频文件写入 MP4 格式
    clip.write_videofile(output_file, codec='libx264')

webm_path = "/home/junpeng.hu/Videos/Webcam/2024-04-12-104441.webm"
webm_path = "/data/hdd1/storage/junpeng/ws_anygrasp/suctionnet-baseline/2024-04-12-110944.webm"
webm_path = "/data/hdd1/storage/junpeng/ws_anygrasp/suctionnet-baseline/2024-04-12-140829.webm"
webm_path = "/data/hdd1/storage/junpeng/ws_anygrasp/suctionnet-baseline/2024-04-12-143158.webm"
webm_path = "/data/hdd1/storage/junpeng/ws_anygrasp/suctionnet-baseline/2024-04-12-144102.webm"
webm_path = "/data/hdd1/storage/junpeng/ws_anygrasp/suctionnet-baseline/2024-04-12-145651.webm"
webm_path = "/data/hdd1/storage/junpeng/ws_anygrasp/suctionnet-baseline/2024-04-12-150542.webm"

basename = os.path.basename(webm_path).split(".")[0]

# 使用函数进行转换
convert_webm_to_mp4(webm_path, '{}.mp4'.format(basename))

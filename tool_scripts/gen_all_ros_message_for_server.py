import os
import sys

if len(sys.argv) < 2:
    print('Usage: python3 gen_all_ros_message_for_server.py <output_dir>')
    exit(1)

ros_dir = '/opt'
# 列出ros_dir下所有*.msg\*.srv文件的绝对路径
msg_files = []
for root, dirs, files in os.walk(ros_dir):
    for file in files:
        if os.path.splitext(file)[1] == '.msg':
            msg_files.append(os.path.join(root, file))
        if os.path.splitext(file)[1] == '.srv':
            msg_files.append(os.path.join(root, file))

cmd = 'rosrun ros_hybrid_sdk ros_hybrid_protoc --server -o ' + sys.argv[1] + ' '
for msg_file in msg_files:
    cmd += '-i ' + msg_file + ' '

os.system(cmd)

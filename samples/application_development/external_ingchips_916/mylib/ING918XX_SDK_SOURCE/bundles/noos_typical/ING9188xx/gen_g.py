with open('apis.json', 'r') as input_file:
    # 打开要写入的文件
    with open('symdefs_zyx.g', 'w') as output_file:
        # 逐行读取输入文件
        for line in input_file:
            # 如果行中包含'{'或'}'，则跳过该行
            if '{' in line or '}' in line:
                continue
            # 去掉双引号和逗号
            line = line.replace('"', '').replace(',', '')
            # 使用split方法分割每行，获取键和值
            parts = line.split(':')
            # 如果分割结果不符合预期，跳过该行
            if len(parts) != 2:
                continue
            key = parts[0].strip()
            value = parts[1].strip()
            # 格式化并写入输出文件
            formatted_line = f'{key} = {value};\n'
            output_file.write(formatted_line)
print("转换完成")

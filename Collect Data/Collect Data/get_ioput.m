clear;
close all;
clc;

% --- 1. 在这里列出你所有的文件名 ---
% 使用一个元胞数组(cell array)来存储所有文件名
file_list = {
    'arduino_log_2025-08-28_16-44-24.txt'
};

% --- 2. 初始化一个空的主矩阵，用于存放所有数据 ---
M = []; % M 将会是最终合并后的大矩阵

% --- 3. 使用 for 循环遍历每一个文件 ---
for i = 1:numel(file_list)
    current_filename = file_list{i}; % 获取当前要处理的文件名
    fprintf('正在处理文件: %s\n', current_filename); % 打印提示信息
    
    fid = fopen(current_filename, 'r');
    if fid < 0
        % 如果某个文件打开失败，打印警告并跳过该文件，继续处理下一个
        warning('无法打开文件: %s。已跳过。', current_filename);
        continue; 
    end
    
    % 使用 textscan 一次性读取当前文件的所有数据
    dataCell = textscan(fid, '%f %f %f %f %f %f', 'Delimiter', ',');
    fclose(fid);
    
    % 将读取到的数据转换为临时矩阵
    % cell2mat转成矩阵，然后转置'，得到每列代表一行数据的格式
    temp_M = cell2mat(dataCell)';
    
    % --- 4. 将当前文件的数据 (temp_M) 水平合并到主矩阵 (M) 中 ---
    if ~isempty(temp_M)
        M = [M, temp_M]; % M 的列数会增加
    end
end

fprintf('所有文件处理完毕。\n');

% --- 5. 对合并后的总矩阵 M 进行后续处理 ---
% 这部分代码保持不变，它会在所有文件数据都加载完毕后执行
if size(M, 2) > 1 
    ud = M(1:3, 1:end);
    yd = M(4:6, 1:end);
else
    disp('所有文件中读取到的数据不足，无法生成 U 和 Y 矩阵。');
end

% 显示最终结果
disp('------------------------------------');
fprintf('从 %d 个文件中总共提取了 %d 条数据。\n', numel(file_list), size(M, 2));
disp('最终合并的矩阵 M 的维度:');
disp(size(M));

% 保存最终的轨迹数据到文件 ---
save('ioput.mat', 'ud', 'yd');

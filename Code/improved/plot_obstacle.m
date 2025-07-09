% 3D障碍物绘制函数
function []=plot_obstacle(x, y, z, r)
    [X, Y, Z] = sphere(15);
    X = r * X + x;
    Y = r * Y + y;
    Z = r * Z + z;
    surf(X, Y, Z, 'FaceColor', 'red', 'FaceAlpha', 0.6, 'EdgeColor', 'none');
end
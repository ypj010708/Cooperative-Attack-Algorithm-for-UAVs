% 3D目标绘制函数
function  [] = plot_target(x, y, z, r)
    [X, Y, Z] = sphere(20);
    X = r * X + x;
    Y = r * Y + y;
    Z = r * Z + z;
    surf(X, Y, Z, 'FaceAlpha', 0.3, 'FaceColor', 'blue', 'EdgeColor', 'none');
end
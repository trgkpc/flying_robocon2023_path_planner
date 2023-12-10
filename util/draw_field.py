import matplotlib.pyplot as plt
import numpy as np

def e(theta):
    return np.array([np.cos(theta), np.sin(theta)])

# コンポーネント
def draw_circle(x, y, radius):
    center = np.array([x, y])
    theta = np.linspace(0, 2*np.pi, 100)
    x,y = (center + radius * e(theta).T).T
    plt.plot(x, y)

def draw_square(x0, y0, x1, y1):
    X = [x0, x0, x1, x1, x0]
    Y = [y0, y1, y1, y0, y0]
    plt.plot(X, Y)

def draw_square_obj(x, y, w, h):
    draw_square(x-w/2, y-h/2, x+w/2, y+h/2)

def draw_line(x0, y0, x1, y1):
    plt.plot([x0, x1],[y0,y1])

# メイン
def draw_field():
    # 等幅設定
    plt.axes().set_aspect('equal', 'datalim')

    # 大枠
    draw_square(-10, 13, 32,-13)
    # ヘリポート
    draw_square_obj(-1.75, 0, 1.5, 1.5)
    draw_square_obj(23.75, 0, 1.5, 1.5)
 
    # ミッションエリア
    draw_line(  0, 13,  0,-13)
    draw_line( 22, 13, 22,-13)
 
    # 物資投下エリア
    draw_square( 10, 6, 15, -6)
 
    # ポール
    r = 0.5
    #draw_circle(25, 6, r)
    #draw_circle(25,-6, r)
    r /= 2
    draw_square_obj(25, 6, r, r)
    draw_square_obj(25,-6, r, r)

    # 高所物資運搬
    draw_square_obj(12.5, 4, 0.5, 0.5)
    draw_square_obj(12.5,-4, 0.5, 0.5)

if __name__ == '__main__':
    draw_field()
    plt.savefig("field.png")
    plt.show()


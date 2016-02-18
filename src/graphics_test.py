from graphics import *
import time

def main():
    win = GraphWin("My Circle", 100, 100)
    for num in range(0,10):
		c = Circle(Point(50,50), 10)
		c.move(10,0)
		c.draw(win)
		time.sleep(.2)
    win.getMouse() # Pause to view result
    win.close()    # Close window when done

main()

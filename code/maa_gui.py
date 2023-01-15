# for testing with Python version 3 use 'tkinter' instead of 'Tkinter' => python version 2.7
import tkinter as tk
from tkinter import *
from tkinter import messagebox
import numpy as np
# pip install pandas //needed to be able to use pandas
import pandas as pd
# for the following this command is needed to install 'sudo apt-get install python-imaging-tk'
from PIL import ImageTk, Image  # to be able to include images

from main import calcForAllTasks




#define some fixed values for the GUI frame
HEIGHT = 450
WIDTH = 700
root = tk.Tk()
root.title("ROS GUI")
canvas = tk.Canvas(root, height=HEIGHT, width=WIDTH)
canvas.pack()


#inititalize a default variable (Euler angle)
inEuler = True
v = IntVar() #for euler/quaternion switch

#create a background picture:
background_image = tk.PhotoImage(file='./documentation/pictures/robot.png', master=root)
background_label = tk.Label(root, image=background_image)
background_label.place(relwidth=1, relheight=1)

image_frame = tk.Frame(root, bg='#00549f', bd=5)
image_frame.place(relx=0.33, rely=0.73, relwidth=0.59, relheight=0.25, anchor='n')




#this function submits csv content, height and width (top right corner)
def visual_output(angle, toolheight, toolwidth ):
	if inEuler==True:
		angle = "euler angle"
	else:
		angle = "quaternion"

	label['text'] = 'csv content: ' + angle +'\n'+ ''.join(
		'Tool-height: ' + str(toolheight) + ' \n'+
		'Tool-width: ' + str(toolwidth)  + ' \n'+
		'.h5-name: ' + h5_entry.get() + ' \n'+
		'.csv-name: ' + csv_entry.get() + ' \n'+
		'\n In case of an error resubmit'
	)




#This function reads the .csv file and creates a numpy array of arrays, each inner array is a row in the .csv file
#The first array (df_array[0]) saves the values of the position of the robot, all further df_array[1]-df_array[n] are values of the tasks
def run_code(csv_path):
	try:
		df = pd.read_csv(csv_path, na_values="?", header=None)
		df_array =  df.to_numpy() #creating a numpy array of arrays

		#integration with main.py:
		#getting all Gui inputs and manipulate the .csv file
		file = h5_entry.get()
		robot_pose = df_array[0]
		df_array_new = np.delete(df_array, (0), axis=0)
		task_poses_global = df_array_new
		tool_height_new = float(tool_height.get())
		tool_width_new = float(tool_width.get())

		#HERE: using main.py methods
		calcForAllTasks(file, robot_pose,task_poses_global,tool_height_new,tool_width_new,20,True,'result.csv',0.11,inEuler)


		#if everything went succesfully:
		messagebox.showinfo("Data processed", "Your results are stored in the result file (.csv and .svg)!")
	except:
		#in case of an error:
		messagebox.showerror("Unable to process", "Something went wrong, please retry!")



#picture popup of "Example of .csv file with euler angle"
def onClick1():
    global csvex1
    csvex1= tk.Toplevel(root)
    csvex1.geometry("650x350")
    csvex1.title("Example of .csv file with euler angle")

    global csveuler
    csveuler = tk.PhotoImage(file='./documentation/pictures/Beispiel_csveuler.png', master=root)
    csveuler_frame = tk.Frame(csvex1, bg='#00549f', bd=5)
    csveuler_frame.place(relx=0.5, rely=0.01, relwidth=0.99, relheight=0.975, anchor='n')
    panelcsv1 = tk.Label(csveuler_frame, image = csveuler)
    panelcsv1.place(relwidth=1, relheight=1, anchor='n')
    panelcsv1.pack()

#picture popup of "Example of .csv file with quaternion"
def onClick2():
    global csvex2
    csvex2= tk.Toplevel(root)
    csvex2.geometry("650x350")
    csvex2.title("Example of .csv file with quaternion")

    global csvquater
    csvquater = tk.PhotoImage(file='./documentation/pictures/Beispiel_csvquat2.png', master=root)
    csvquater_frame = tk.Frame(csvex2, bg='#00549f', bd=5)
    csvquater_frame.place(relx=0.5, rely=0.01, relwidth=0.99, relheight=0.975, anchor='n')
    panelcsv2 = tk.Label(csvquater_frame, image = csvquater)
    panelcsv2.place(relwidth=1, relheight=1, anchor='n')
    panelcsv2.pack()

#picture popup of "Correct way to measure the tool"
def onClick3():
    global csvex3
    csvex3= tk.Toplevel(root)
    csvex3.geometry("425x250")
    csvex3.title("Correct way to measure the tool")

    global toolex
    toolex = tk.PhotoImage(file='./documentation/pictures/tool_example2.png', master=root)
    toolex_frame = tk.Frame(csvex3, bg='#00549f', bd=5)
    toolex_frame.place(relx=0.5, rely=0.01, relwidth=0.99, relheight=0.975, anchor='n')
    paneltool = tk.Label(toolex_frame, image = toolex)
    paneltool.place(relwidth=1, relheight=1, anchor='n')
    paneltool.pack()

	
#Info text top left
first_text_frame = tk.Frame(root, bg='#00549f', bd=5)
first_text_frame.place(relx=0.14, rely=0.03, relwidth=0.215, relheight=0.3, anchor='n')

label = tk.Label(first_text_frame, font=("Ariel", 8))
label['text'] = 'This GUI is used for an \n automated calculation \n of reachability. \n \n Enter your data, then \n press the "RUN CODE" \n button to generate \n the reachability map'
label.place(relwidth=1, relheight=1)


#Create the entering frame for euler/quaternion/tool-height/tool-width
entering_frame = tk.Frame(root, bg='#00549f', bd=5)
entering_frame.place(relx=0.45, rely=0.03, relwidth=0.35, relheight=0.3, anchor='n')

#create Labels, so the user knows what to input
# tk.Label(entering_frame, text= 'csv content: ', font=("Ariel", 10), bg='#DFE8F1').place(relx=0, rely=0, relwidth=0.49, relheight=0.31)
tk.Label(entering_frame, text= 'Tool-height (in m): ', font=("Ariel", 9), bg='#DFE8F1').place(relx=0, rely=0.345, relwidth=0.49, relheight=0.31)
tk.Label(entering_frame, text= 'Tool-width (in m): ', font=("Ariel", 9), bg='#DFE8F1').place(relx=0, rely=0.68, relwidth=0.49, relheight=0.31)


#XOR switches (toop middle of GUI)
def set_euler():
	global inEuler
	inEuler = True

def set_quaternion():
	global inEuler
	inEuler = False

#create entries (XOR switch):
button_csv_content_euler = Radiobutton(entering_frame,text="euler angle", font=("Ariel", 10), variable=v, value=1, command=lambda:set_euler())
button_csv_content_euler.place(relx= 0, rely=0, relwidth=0.49, relheight=0.31)
button_csv_content_quaternion = Radiobutton(entering_frame,text="quaternion", font=("Ariel", 10), variable=v, value=2, command=lambda:set_quaternion())
button_csv_content_quaternion.place(relx= 0.51, rely=0, relwidth=0.49, relheight=0.31)

#create user input entries:
tool_height = tk.Entry(entering_frame, font=("Ariel", 10))
tool_height.place(relx= 0.51,  rely=0.345, relwidth=0.49, relheight=0.31)
tool_width = tk.Entry(entering_frame, font=("Ariel", 10))
tool_width.place(relx= 0.51, rely=0.68, relwidth=0.49, relheight=0.31)

#creat a button for submition (top right middle button):
button_value_submition = tk.Button(root, text="Submit values", font=("Ariel", 16),
	command=lambda: visual_output(
		str(inEuler), tool_height.get(),
		tool_width.get()
	)
)
button_value_submition.place(relx=0.8, rely=0.38, relwidth=0.3, relheight=0.18, anchor= 'n')

#Text frame for .csv entry:
frame_csv = tk.Frame(root, bg='#00549f', bd=5)
frame_csv.place(relx=0.33, rely=0.57, relwidth=0.59, relheight=0.09, anchor='n')
csv_label = tk.Label(frame_csv, text= 'Create a UTF-8 .csv file, as presented below \n and enter the file-name here: ', font=("Ariel", 9), bg='#DFE8F1')
csv_label.place(relwidth=1, relheight=1)

#.csv entry:
frame_csv_entry = tk.Frame(root, bg='#00549f', bd=5)
frame_csv_entry.place(relx=0.33, rely=0.65, relwidth=0.59, relheight=0.07, anchor='n')
csv_entry = tk.Entry(frame_csv_entry, font=("Ariel", 10))
csv_entry.place(relwidth=1, relheight=1)


#Text frame for .h5 entry:
frame_h5info = tk.Frame(root, bg='#00549f', bd=5)
frame_h5info.place(relx=0.33, rely=0.49, relwidth=0.59, relheight=0.07, anchor='n')
h5info_label = tk.Label(frame_h5info, text= 'To create a .h5 file please follow the instructions in README', font=("Ariel", 9), bg='#DFE8F1')
h5info_label.place(relwidth=1, relheight=1)

frame_h5 = tk.Frame(root, bg='#00549f', bd=5)
frame_h5.place(relx=0.33, rely=0.34, relwidth=0.59, relheight=0.08, anchor='n')
h5_label = tk.Label(frame_h5, text= 'Enter the .h5 file for the reachability map of the robot here: ', font=("Ariel", 9), bg='#DFE8F1')
h5_label.place(relwidth=1, relheight=1)

#.h5 entry:
frame_h5_entry = tk.Frame(root, bg='#00549f', bd=5)
frame_h5_entry.place(relx=0.33, rely=0.41, relwidth=0.59, relheight=0.07, anchor='n')
h5_entry = tk.Entry(frame_h5_entry, font=("Ariel", 10))
h5_entry.place(relwidth=1, relheight=1)


#Bottom right button to run the final code:
button_run_code= tk.Button(root, text="Run code", font=("Ariel", 14), bg= '#FF3C3C',
	command=lambda: run_code(csv_entry.get())
	)
button_run_code.place(relx=0.8, rely=0.86, relwidth=0.3, relheight=0.118, anchor= 'n')




#Bottom right note above the 'Run code' button
frame_note = tk.Frame(root, bg='#00549f', bd=5)
frame_note.place(relx=0.8, rely=0.62, relwidth=0.3, relheight=0.18, anchor= 'n')
note_label = tk.Label(frame_note, text= 'Once you entered \n all data correctly \n press the button below', font=("Ariel", 10), bg='#DFE8F1')
note_label.place(relwidth=1, relheight=1)





#Top right corner default text settings:
side_frame = tk.Frame(root, bg='#34495E', bd=5)
side_frame.place(relx=0.8, rely=0.03, relwidth=0.3, relheight=0.3, anchor='n')
label = tk.Label(side_frame)
label['text'] = '[Parameters will \n be displayed here \n once submited]' #text will change once button is pressed
label.place(relwidth=1, relheight=1)



#Bottom buttons for pictures [euler/quaternion/tool_example]
button_csveuler= tk.Button(image_frame, text="Example of \n.csv file \nwith euler angle", font=("Ariel",10), bg= '#DFE8F1',
	command=lambda: onClick1()
	)
button_csveuler.place(relx=0.165, rely=0.01, relwidth=0.33, relheight=0.98, anchor= 'n')

button_csvquaternion= tk.Button(image_frame, text="Example of \n.csv file \nwith quaternion", font=("Ariel",10), bg= '#DFE8F1',
	command=lambda: onClick2()
	)
button_csvquaternion.place(relx=0.504, rely=0.01, relwidth=0.325, relheight=0.98, anchor= 'n')

button_exampletool= tk.Button(image_frame, text="Example of\n tool height \nand width", font=("Ariel",10), bg= '#DFE8F1',
	command=lambda: onClick3()
	)
button_exampletool.place(relx=0.84, rely=0.01, relwidth=0.32, relheight=0.98, anchor= 'n')


#This must be at the end of the Gui
root.mainloop()
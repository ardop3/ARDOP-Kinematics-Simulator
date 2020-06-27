# import necessary Libraries

import tkinter
from tkinter import ttk
import numpy as np
import math
from numpy.linalg import inv
import serial
import matplotlib.pyplot as plt 
from PIL import ImageTk,Image  
import sys, select, termios, tty



window = tkinter.Tk()

window.title("ARDOP Kinematics Simulator ")

window.geometry('500x500')

Label_main_1 = ttk.Label(window, text=" Enter Robot Description"  , font='Aerial 10 bold')
Label_main_1.grid(column=0, row=0)



## ------------------------------------------------ ##

## ARDOP Description window

description_window = tkinter.Toplevel()
description_window.title("ARDOP Description Window ")
canvas = tkinter.Canvas(description_window, width = 400, height = 400)  
canvas.pack()  
img = ImageTk.PhotoImage(Image.open("ARDOP.png"))  
canvas.create_image(50, 50,anchor = tkinter.NW, image=img) 


## ------------------------------------------------ ##


label_a2 = ttk.Label(window, text="a2" )

label_a2.grid(column=0, row=3)

a_2 = tkinter.DoubleVar()

txt_a2 = ttk.Entry(window,width=26 , textvariable = a_2 )


txt_a2.grid(column=1, row=3)


## ------------------------------------------------ ##



label_a3 = ttk.Label(window, text="a3" )

label_a3.grid(column=0, row=6)

a_3 = tkinter.DoubleVar()

txt_a3 = ttk.Entry(window,width=26 , textvariable = a_3 )



txt_a3.grid(column=1, row=6)

## ------------------------------------------------ ##



label_a4 = ttk.Label(window, text="a4" )

label_a4.grid(column=0, row=9)

a_4 = tkinter.DoubleVar()


txt_a4 = ttk.Entry(window,width=26 , textvariable = a_4 )

txt_a4.grid(column=1, row=9)


## ------------------------------------------------ ##



label_shoulder = ttk.Label(window, text="shoulder" )

label_shoulder.grid(column=0, row=12)

shoulder = tkinter.DoubleVar()


txt_shoulder = ttk.Entry(window,width=26 , textvariable = shoulder )

txt_shoulder.grid(column=1, row=12)


## ------------------------------------------------ ##




label_neck = ttk.Label(window, text="neck" )

label_neck.grid(column=0, row=15)

neck = tkinter.DoubleVar()


txt_neck = ttk.Entry(window,width=26 , textvariable = neck )

txt_neck.grid(column=1, row=15)




## ------------------------------------------------ ##





## ------------------------------------------------ ##


## ------------------------------------------------ ##

Label_main_2 = ttk.Label(window, text="Enter Co-ordinates in camera Frame" ,  font='Aerial 10 bold' )
Label_main_2.grid(column=0, row= 40 )



## ------------------------------------------------ ##




label_pixel_u = ttk.Label(window, text="X" )

label_pixel_u.grid(column=0, row=44)

pixel_u = tkinter.DoubleVar()


txt_pixel_u = ttk.Entry(window,width=26 , textvariable = pixel_u )

txt_pixel_u.grid(column=1, row=44)


## ------------------------------------------------ ##



label_pixel_v = ttk.Label(window, text="Y" )

label_pixel_v.grid(column=0, row=48)

pixel_v = tkinter.DoubleVar()


txt_pixel_v = ttk.Entry(window,width=26 , textvariable = pixel_v )

txt_pixel_v.grid(column=1, row=48)



## ------------------------------------------------ ##


label_depth = ttk.Label(window, text=" Z" )

label_depth.grid(column=0, row=52)


depth = tkinter.DoubleVar()


txt_depth = ttk.Entry(window,width=26 , textvariable = depth )

txt_depth.grid(column=1, row=52)



## ------------------------------------------------ ##



def forward(new_angle):
     

     theta0 = new_angle.item(0)  
     theta1 = new_angle.item(1) 
     theta2 = new_angle.item(2) 
     
     l= a3*math.sin(theta1) +a4*math.sin(theta1+theta2)
     x= l*math.sin(theta0)
     y = l*math.cos(theta0)
     z = (a3*math.cos(theta1) + a4*math.cos(theta1+theta2)) 

     co = np.array([x,y,z])
     #print("forward")
     return co

## ------------------------------------------------ ##

def inverse(x,y,z):

    error = 0 
    
    theta0 = math.atan(x/y) + error 

    r2 = math.sqrt(x*x + y*y)
    r1 = z#+a1
    phi2 = math.atan(r2/r1)
    r3 = math.sqrt(r1*r1 + r2*r2)
    p = a3*a3 +r3*r3 -a4*a4
    q = 2*a3*r3
    phi1 = math.acos(p/q)
    theta1 = phi2 - phi1  + error  

    r = a3*a3+a4*a4-r3*r3
    s= 2*a3*a4

    phi3 = math.acos(r/s)

    theta2 = pi - phi3  + error 
    deg = np.array([theta0,theta1,theta2])
    #print("inv")
    return deg

## ------------------------------------------------ ##

def delta(target_co,cur_co):

    i1 = np.array([target_co - cur_co])
    i = i1.transpose()
    #print("delta")
    return i 

## ------------------------------------------------ ##

def inverse_jacobian(new_angle):

    a0 = new_angle.item(0)
    a1 = new_angle.item(1)
    a2 = new_angle.item(2)

    jx0 = (a3*math.sin(a1) +a4*math.sin(a1+a2)) * math.cos(a0)
    jx1 = (a3*math.cos(a1) + a4*math.cos(a1+a2))*math.sin(a0)
    jx2 = (a4*math.cos(a1 +a2))*math.cos(a0)

    jy0 = -(a3*math.sin(a1) +a4*math.sin(a1+a2))*math.sin(a0)
    jy1 = (a3*math.cos(a1) +a4 *math.cos(a1+a2)) *math.cos(a0)
    jy2 = (a4*math.cos(a1 +a2)) *math.cos(a0)

    jz0 = 0
    jz1 = (-a3*math.sin(a1) -a4*math.sin(a1+a2))
    jz2 = -a4*math.sin(a1+a2)

    jacobian = np.array([[jx0,jx1,jx2],[jy0,jy1,jy2],[jz0,jz1,jz2]])
    inv_jacobian = inv(jacobian)
    #print("inv_jaco")
    return inv_jacobian



## ------------------------------------------------ ##

def collect_time():

	t0 = float(txt_t0.get())
	tf = float(txt_tf.get())
	display_velocity_plots ( ang0 , ang1, ang2 , t0, tf)


## ------------------------------------------------ ##

## ARDOP Trajectory Planning window

def display_results( theta_0 , theta_1 , theta_2):

	res_window = tkinter.Tk()

	res_window.title("ARDOP Trajectory Planning Simulator ")

	res_window.geometry('500x500')

	h = ( theta_0 , theta_1 , theta_2 )

	global txt_t0
	global txt_tf


	Label_vel = ttk.Label(res_window, text="Inverse Kinematics Planning Result " ,  font='Aerial 10 bold' )
	Label_vel.grid(column=0, row= 0 )



	result = ttk.Label(res_window, text= "theta0" ,  font='Aerial 10 bold' )
	result.grid(column=0, row=1)

	result = ttk.Label(res_window, text= theta_0 )
	result.grid(column=1, row=1)


	result = ttk.Label(res_window, text= "theta1" ,  font='Aerial 10 bold' )
	result.grid(column=0, row=2)

	result = ttk.Label(res_window, text= theta_1 )
	result.grid(column=1, row=2)


	result = ttk.Label(res_window, text= "theta2" ,  font='Aerial 10 bold')
	result.grid(column=0, row=3)

	result = ttk.Label(res_window, text= theta_2 )
	result.grid(column=1, row=3)	


	Label_vel = ttk.Label(res_window, text="Enter Time For Trajectory Planning " ,  font='Aerial 10 bold' )
	Label_vel.grid(column=0, row= 15 )


	label_t0 = ttk.Label(res_window, text="  Enter Start Time " )
	label_t0.grid(column=0, row=18)
	t_0 = tkinter.DoubleVar()
	txt_t0 = ttk.Entry(res_window,width=26 , textvariable = t_0 )
	txt_t0.grid(column=1, row=18)

	label_tf = ttk.Label(res_window, text="  Enter Stop Time " )
	label_tf.grid(column=0, row=21)
	t_f = tkinter.DoubleVar()
	txt_tf= ttk.Entry(res_window,width=26 , textvariable = t_f )
	txt_tf.grid(column=1, row=21)

	global t0
	global tf

	




	btn = ttk.Button(res_window, text="Submit  ", command= collect_time)

	btn.grid(column=0, row=60)






## ------------------------------------------------ ##


def time_domain (t_1, t_2 ):

	t0 =t_1
	tf = t_2

	a_1 = t0**3
	a_2 = t0**2
	a_3 =  t0
	a_4 = 1

	a_5 = 3*t0**2
	a_6 = 2*t0
	a_7 =1
	a_8 =0

	a_9 =tf**3
	a_10 = tf**2
	a_11 = tf
	a_12 =1

	a_13 = 3*tf**2
	a_14 =2*tf
	a_15 =1
	a_16 =0

	A = np.array([  [a_1 , a_2 , a_3 , a_4 ] ,     [a_5 , a_6 , a_7 , a_8 ] , [a_9 , a_10 , a_11 , a_12 ],  [a_13 , a_14 , a_15 , a_16 ]    ])
	inv_A = inv(A)
	
	return inv_A


## ------------------------------------------------ ##


def angle_matrix( a,b,c):

	q0 = 0
	q1 = 0
	q2 = 0
	q3 = 0
	q4 =0
	q5= 0


	q6 = 0
	q7 = 0
	q8 = 0
	q9 = 0
	q10 =0
	q11= 0


	q12 = a
	q13 = b
	q14 = c
	q15 = 15
	q16=  25
	q17= 0


	q18 = 0
	q19 = 0
	q20 = 0
	q21 = 0
	q22 =0
	q23= 0

	angle_B = np.array ( [    [q0,q1,q2,q3,q4,q5]  , [q6,q7,q8,q9,q10,q11] , [q12,q13,q14,q15,q16,q17] , [q18,q19,q20,q21,q22,q23]])
	

	return angle_B


## ------------------------------------------------ ##


def plot_velocity(param):

	vel_angle0 =[]
	vel_angle1 =[]
	vel_angle2 =[]
	vel_angle3 =[]
	vel_angle4 =[]
	vel_angle5 =[]

	time_vel = []

	for t1 in range(15):

		x1 = 3*param[0][0]*(t1**2) + 2*param[1][0]*t1   + param[2][0]
		x2 = 3*param[0][1]*(t1**2) + 2*param[1][1]*t1   + param[2][1]
		x3 = 3*param[0][2]*(t1**2) + 2*param[1][2]*t1  + param[2][2]
		x4 = 3*param[0][3]*(t1**2) + 2*param[1][3]*t1   + param[2][3]
		x5 = 3*param[0][4]*(t1**2) + 2*param[1][4]*t1   + param[2][4]
		x6 = 3*param[0][5]*(t1**2) + 2*param[1][5]*t1   + param[2][5]

		vel_angle0.append(x1)
		vel_angle1.append(x2)
		vel_angle2.append(x3)
		vel_angle3.append(x4)
		vel_angle4.append(x5)
		vel_angle5.append(x6)

		time_vel.append(t1)


	axs[0].plot(vel_angle0 , time_vel ,  label = "vel_theta0")
	axs[0].plot(vel_angle1 , time_vel ,  label = "vel_theta1")
	axs[0].plot(vel_angle2 , time_vel ,  label = "vel_theta2")
	axs[0].plot(vel_angle3 , time_vel ,  label = "vel_theta3")
	axs[0].plot(vel_angle4 , time_vel ,  label = "vel_theta4")
	axs[0].plot(vel_angle5 , time_vel ,  label = "vel_theta5")

	plt.xlabel('velocity')
	plt.ylabel('time')
	plt.legend()
	plt.show()





## ------------------------------------------------ ##


def plot_angle(param , t_start ,  t_end):
	angle0 = []
	angle1 = []
	angle2 = []
	angle3 = []
	angle4 = []
	angle5 = []

	time = []

	t_start_1 = int(t_start)
	t_end_1 = int(t_end)

	for t in range (t_start_1 , t_end_1 ):
		p = param[0][0]*(t**3) + param[1][0]*(t**2) + param[2][0]*(t) + param[3][0]
		q = param[0][1]*(t**3) + param[1][1]*(t**2) + param[2][1]*(t) + param[3][1]
		r = param[0][2]*(t**3) + param[1][2]*(t**2) + param[2][2]*(t) + param[3][2]
		s = param[0][3]*(t**3) + param[1][3]*(t**2) + param[2][3]*(t) + param[3][3]
		l3 = param[0][4]*(t**3) + param[1][4]*(t**2) + param[2][4]*(t) + param[3][4]
		u =param[0][5]*(t**3) + param[1][5]*(t**2) + param[2][5]*(t) + param[3][5]

		angle0.append(p)
		angle1.append(q)
		angle2.append(r)
		angle3.append(s)
		angle4.append(l3)
		angle5.append(u)

		time.append(t)


	vel_angle0 =[]
	vel_angle1 =[]
	vel_angle2 =[]
	vel_angle3 =[]
	vel_angle4 =[]
	vel_angle5 =[]

	time_vel = []

	for t1 in range(t_start_1 , t_end_1 ):

		x1 = 3*param[0][0]*(t1**2) + 2*param[1][0]*t1   + param[2][0]
		x2 = 3*param[0][1]*(t1**2) + 2*param[1][1]*t1   + param[2][1]
		x3 = 3*param[0][2]*(t1**2) + 2*param[1][2]*t1  + param[2][2]
		x4 = 3*param[0][3]*(t1**2) + 2*param[1][3]*t1   + param[2][3]
		x5 = 3*param[0][4]*(t1**2) + 2*param[1][4]*t1   + param[2][4]
		x6 = 3*param[0][5]*(t1**2) + 2*param[1][5]*t1   + param[2][5]

		vel_angle0.append(x1)
		vel_angle1.append(x2)
		vel_angle2.append(x3)
		vel_angle3.append(x4)
		vel_angle4.append(x5)
		vel_angle5.append(x6)

		time_vel.append(t1)


	axs[0].plot(vel_angle0 , time_vel ,  label = "vel_theta0")
	axs[0].plot(vel_angle1 , time_vel ,  label = "vel_theta1")
	axs[0].plot(vel_angle2 , time_vel ,  label = "vel_theta2")
	axs[0].plot(vel_angle3 , time_vel ,  label = "vel_theta3")
	axs[0].plot(vel_angle4 , time_vel ,  label = "vel_theta4")
	axs[0].plot(vel_angle5 , time_vel ,  label = "vel_theta5")

	plt.xlabel('velocity')
	plt.ylabel('time')
	plt.legend()
	#plt.show()



	axs[1].plot(angle0 , time ,  label = "theta0")
	axs[1].plot(angle1 , time ,  label = "theta1")
	axs[1].plot(angle2 , time ,  label = "theta2")
	axs[1].plot(angle3 , time ,  label = "theta3")
	axs[1].plot(angle4 , time ,  label = "theta4")
	axs[1].plot(angle5 , time ,  label = "theta5")

	plt.xlabel('angle')
	plt.ylabel('time')
	plt.legend()
	plt.show()




## ------------------------------------------------ ##

def display_velocity_plots( theta_0_0 , theta_1_0 , theta_2_0 , ton, toff):


	inverse_A = time_domain(ton, toff)
	B = angle_matrix( theta_0_0 , theta_1_0 , theta_2_0 )


	parameters = np.matmul(inverse_A, B)
	global fig, axs 
	fig, axs = plt.subplots(2)

	plot_angle(parameters , ton , toff)

	#plot_velocity(parameters)


	print(parameters)


## ------------------------------------------------ ##




def generate_ik( A2 , A3, A4, X, Y,Z):

		global a3,a4
		a3 = A3
		a4 =A4
		a1 =5  
		          ## link lengths will be updated
		global pi
		pi =3.14



		y = Y
		z = Z
		x = X


		target_co = np.array([x,y,z])
		angle = inverse(x,y,z) # determine approx value
		new_angle = np.array([angle])

		#print(new_angle)
		for k in range(300):

		    cur_co = forward(new_angle)
		    diff_co = delta(target_co, cur_co)
		    diff_angle1 = inverse_jacobian(new_angle)
		    diff_angle = np.matmul(diff_angle1,diff_co)
		    #diff_angle = diff_angle1 * diff_co
		    new_angle = np.add(new_angle,diff_angle)
		    #new_angle = new_angle + diff_angle
		    print(new_angle[0,0]*180/pi , new_angle[0,1]*180/pi, new_angle[0,2]*180/pi)

		global ang0 ,ang1, ang2
		ang0 = new_angle[0,0]*180/pi
		ang1 = new_angle[0,1]*180/pi
		ang2 = new_angle[0,2]*180/pi 
		display_results ( new_angle[0,0]*180/pi ,new_angle[0,1]*180/pi , new_angle[0,2]*180/pi )
		#display_velocity_plots(new_angle[0,0]*180/pi ,new_angle[0,1]*180/pi , new_angle[0,2]*180/pi  )




## ------------------------------------------------ ##




def robot_description():
	#print(txt_a2.get() , txt_a3.get() , txt_a4.get() , txt_shoulder.get())


	link_a2 = float(txt_a2.get())
	link_a3 = float(txt_a3.get())
	link_a4 = float(txt_a4.get())
	co_x =  float(txt_pixel_u.get())
	co_y = float(txt_pixel_v.get())
	co_z = float(txt_depth.get())
	generate_ik(link_a2 ,link_a3, link_a4 ,co_x , co_y, co_z  )




btn = ttk.Button(window, text="Submit  ", command= robot_description)

btn.grid(column=0, row=60)




window.mainloop()

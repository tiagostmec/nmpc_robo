#!/usr/bin/env python
# coding=utf-8


############ IMPORTAÇÃO DE BIBLIOTECAS E INICIALIZAÇÃO DE ALGUMAS VARIÁVELS #########################
import rospy, cv2, cv_bridge, numpy, sys
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from random import randint
import numpy as np
from roslib import message
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
from ar_track_alvar_msgs.msg import AlvarMarkers
from nmpc_robo.srv import *
xr=None
yr=None
thr=None
z = None
roll = None
pitch = None
yaw = None
altitudeZ = 0
integralZ = 0
last_erroZ = 0
distH = 0
vel_linear = 0
integral = 0
ptime = 0
last_erro = 0
dt = 0.1
font = cv2.FONT_HERSHEY_COMPLEX
#flagalt =0
#########################################################################################################

class Follower:

        ##################### INICIALIZAÇÃO DO NÓ DE CONTROLE #######################
        rospy.init_node('line_follower')

        #############################################################################

        def __init__(self):

                ######################################## INICIALIZAÇÃO DE VARIÁVEIS #################################
                self.angulorobo = 0
                self.soma = 0
                self.z = 0
                self.ze = 0
                self.lastz = 0
                self.flag = 0
                self.flag1 = 0
                self.lastrho = 0
                self.aux = 0
                self.xfut = 0
                self.yfut = 0
                self.cont = 0
                self.erro = 0
                self.minimo = 0
                self.last_time = 0
                self.ptime = 0
                self.bridge = cv_bridge.CvBridge()
                self.r = 0 # Velocidade Angular (Yaw)
                self.r_atual = 0
                self.r_anterior = 0
                self.integral = 0
                self.last_erro = 0
                self.derivative = 0
                self.integral1 = 0
                self.last_erro1 = 0
                self.derivative1 = 0
                self.amostras = 0
                self.voltas = 0
                self.soma_z = 0
                self.media_z = 0
                self.soma_time = 0
                self.media_time = 0
                self.iter = 1
                #self.flagalt = 0
                

                cv2.namedWindow("window", 1)

                ############################################## TÓPICOS SUBSCRITOS ##################################
                
                self.image_sub = rospy.Subscriber('/bebop2/image_raw',Image, self.image_callback)
                
                #self.image_sub = rospy.Subscriber('camera/rgb/image_raw',Image, self.image_callback)

                self.cmd_vel_pub = rospy.Publisher('/bebop2/cmd_vel',Twist, queue_size=1)

                self.twist = Twist()
                ##rospy.Subscriber('/camera/depth/image_raw',Image,self.get_distance)

                rospy.Subscriber('/odometry/filtered',Odometry,self.callback_odom)
                self.Odom = Odometry ()

                rospy.Subscriber('/ar_pose_marker',AlvarMarkers,self.callback_alvar)
                #rospy.Subscriber('/camera/depth/image_raw',Image,self.get_distance)
                #rospy.Subscriber("/camera/depth/points", PointCloud2, self.callback_kinect)

                #self.otimiza = self.callback_client(self.omega,self.ze,self.thetar)
                ########################################################################################################

        ############################# CONTROLE PID PARA ALTITUDE #######################################################
        def pidZ(self, erro, kp, ki, kd):
            global integralZ
            global last_erroZ
            integralZ += erro * 0.1
            derivative = (erro - last_erroZ)/0.1
            last_erroZ = erro
            return kp * erro + ki * integralZ + kd * derivative

############################ CONV #########

       
        ###################################################################################################################
        ############################### ALVAR TAG #########################################################################
        def callback_alvar(self,msg):
            global xm
            xm = msg.markers
        ###################################################################################################################
        def sleep(t):
                try:
                        rospy.sleep(t)
                except:
                        pass

        ######################################## PID PARA TESTES ###########################################################
        def pid(self, erro, kp, ki, kd,dt):

            self.integral += erro
            self.integral = integral*dt
            self.derivative = (erro - last_erro)/dt
            self.last_erro = erro
            return kp * erro + ki * self.integral + kd * self.derivative
        
        ##################################### FUNÇÃO PARA O CÁLCULO DE TRAJETÓRIA VIRTUAL CASO A LINHA NÃO SEJA ENCONTRADA #####################
        def get_curva(self,x1,y1,x2,y2,x3,y3):

                denom = (x1 - x2) * (x1 - x3) * (x2 - x3)
                denom = float(denom)
                A     = (x3 * (y2 - y1) + x2 * (y1 - y3) + x1 * (y3 - y2)) / denom
                B     = (x3*x3 * (y1 - y2) + x2*x2 * (y3 - y1) + x1*x1 * (y2 - y3)) / denom
                C     = (x2 * x3 * (x2 - x3) * y1 + x3 * x1 * (x3 - x1) * y2 + x1 * x2 * (x1 - x2) * y3) / denom
                A = float(A)
                B = float(B)
                C = float (C)
                #print ('A',A)
                #print ('B',B)
                #print ('C',C)
                #print ('denom',denom)
                x300 = A*240*240 + B*240 + C
                x280 = A*220*220 + B*220 + C
                x320 = A*260*260 + B*260 + C
                x300 = int(x300)
                x280 = int(x280)
                x320 = int(x320)
                return x300,x280,x320 

        ########################################################################################################################################
        
        ################################# FUNÇÃO PARA O CÁLCULO DO TAMANHO DA LINHA ENCONTRADA EM CASO DA FALHA ##################################
        def tamanho_linha (self,mask,image):
                _, contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                if len(contours) > 0 :
                        for cnt in contours:
                                approx = cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt, True), True)
                                #print(len(approx))
                                #cv2.drawContours(image, [approx], 0, (0), 5)
                                x = approx.ravel()[0]
                                y = approx.ravel()[1]

                                box = cv2.boundingRect(cnt)
                                #print(len(contours))
                                tamanho_janela = box[3]/3                      
                else :
                        tamanho_janela = 1


                return tamanho_janela

        #########################################################################################################################################

        ############################################## FUNÇÃO QUE EXTRAI AS INFORMAÇÕES DE COMPRIMENTO E ALTURA DA IMAGEM DO KINECT #####################################################
        def callback_kinect(self,data) :
                # pick a height
                height =  int (data.height / 2)
                # pick x coords near front and center
                middle_x = int (data.width / 2)
                # examine point
                middle = self.read_depth(middle_x,height,data)
                # do stuff with middle
        #########################################################################################################################################################################################

        ################################### FUNÇÃO QUE ADQUIRE OS PONTOS DA CÂMERA RGB-D E GERA UM VETOR (SELF.B) CONTENDO A PROFUNFIFADE PARA CADA PIXEL (0 - WIDTH DA CÂMERA) NO CENTRO DA CÂMERA (HEIGHT 300)
        def read_depth(self,width, height,data) :

                self.int_data = []
                if (height >= data.height) or (width >= data.width) :
                        return -1
                for i in range(2*width):
                        
                        data_out = pc2.read_points(data, field_names=None, skip_nans=False, uvs=[[i, height]])
                        #int_data1.append(next(data_out))
                        
                        #print('width',height)
                        self.int_data.append(next(data_out)[2]) 
                        #print(self.int_data)
                        #print(len(self.int_data))
                #rospy.loginfo("int_data " + str(int_data1[400]))
                #print('data',self.int_data[400])
                
                #print('xr',xr)        
                self.b = np.where(np.isnan(self.int_data), 5, self.int_data) # b : vetor contendo todas as profundidades para uma linha
                #print (self.b)
                #print('data',self.b[width])

        ############################################## FUNÇÃO QUE CHAMA O OTIMIZADOR ##################################################################
        def callback_client(self,roll,pitch,yaw,ze,phir,r,altitudeZ,vel_linear):
                rospy.wait_for_service('otimizador')
                try:
                        self.otimizador = rospy.ServiceProxy('otimizador', otimizador)
                        self.resposta = self.otimizador(roll,pitch,yaw,self.ze,self.phir,self.r,distH,vel_linear)
                        #print('resposta:', self.resposta.u)
                        return self.resposta.u,self.resposta.v 
                except rospy.ServiceException, e:
                        print "Falhou de novo haha: %s" %e
        ################################################################################################################################################

        ############################################################## FUNÇÃO QUE EXTRAI A ODOMETRIA DO DRONE ##################################################
        def callback_odom(self,msg):
            global xr
            global yr
            global thr
            global roll
            global pitch
            global yaw  
            global tf
            global r
            global ze
            global thetar
            global flagalt





                        
            xr=(-1)*msg.pose.pose.position.y
            yr=msg.pose.pose.position.x
            thr = np.arctan2(2*msg.pose.pose.orientation.w*msg.pose.pose.orientation.z,1-2*msg.pose.pose.orientation.z*msg.pose.pose.orientation.z) 


            quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
            )
            euler = euler_from_quaternion(quaternion)
            global altitudeZ
            altitudeZ = msg.pose.pose.position.z
            roll = euler[0]
            pitch = euler[1]
            yaw = euler[2]        
        ####################################################################################################################################################

        ################################################### FUNÇÃO QUE EXTRAI INFORMAÇÕES DA CÂMERA ##############################################
        def get_distance(self,img):
                
                self.bridge = cv_bridge.CvBridge()
                cv_image = self.bridge.imgmsg_to_cv2(img,'32FC1')
                cv_image = np.array(cv_image, dtype=np.float32)
                
                #print ('dist',cv_image)
                return cv_image

        ########################################################### FUNÇÃO QUE EXECUTA O CONTROLE ###################################################################
              
        def image_callback(self, msg):
                    
                ############################### TRANSFORMAÇÃO DA IMAGEM PARA SER TRABALHADA PELO OPEN CV #########################################
                image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')

                ################ Transformação da imagem de RGB para HSV ###############################################
                hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
                
                ###################### ETAPA DE CLASSIFICAÇÃO DA IMAGEM - CRIAÇÃO DAS MÁSCARAS BINÁRIAS ########################################
                ################## CIRCUITO VERDE #################
                lower_yellow = numpy.array([16, 100, 100]) #16 100 100 e 36 250 250 220 220 
                upper_yellow = numpy.array([46, 250,250])
                
                ##################### LINHA DE TRANSMISSÃO PRETA #################
                #lower_yellow = numpy.array([0, 0, 0])
                #upper_yellow = numpy.array([10,10,10])
                ##################################################################

                ####################### A IMAGEM EM HSV É PASSADA PELA ETAPA DE CLASSIFICÃO GERANDO UMA IMAGEM BINÁRIA ONDE O BRANCO É A LINHA A SER SEGUIDA E O PRETO É A ÁREA DESCARTADA ######################
                mask = cv2.inRange(hsv, lower_yellow, upper_yellow) # Pr
                mask1 = cv2.inRange(hsv, lower_yellow, upper_yellow) # P1
                mask2 = cv2.inRange(hsv, lower_yellow, upper_yellow) # P2
                mask3 = cv2.inRange(hsv, lower_yellow, upper_yellow) # PCurvaSuperior1
                mask4 = cv2.inRange(hsv, lower_yellow, upper_yellow) # PCurvaSuperior2
                mask5 = cv2.inRange(hsv, lower_yellow, upper_yellow) # PCurvaSuperior3
                mask6 = cv2.inRange(hsv, lower_yellow, upper_yellow) # PCurvaInferior1
                mask7 = cv2.inRange(hsv, lower_yellow, upper_yellow) # PCurvaInferior2
                mask8 = cv2.inRange(hsv, lower_yellow, upper_yellow) # PCurvaInferior3
                mask9 = cv2.inRange(hsv, lower_yellow, upper_yellow)
                mask10 = cv2.inRange(hsv, lower_yellow, upper_yellow)
                mask11 = cv2.inRange(hsv, lower_yellow, upper_yellow)
                mask12 = cv2.inRange(hsv, lower_yellow, upper_yellow)
                mask13 = cv2.inRange(hsv, lower_yellow, upper_yellow)

                #print(mask9)


                #cv2.imshow("shapes",image)
                #cv2.imshow("Threshold", mask)


                ################################################### CRIAÇÃO DAS JANELAS UTILIZADAS CALCULAR OS PONTOS DE INTERESSE #####################################################################

                ########## EXTRAI O FORMATO DA IMAGEM

                h, w, d = image.shape

                ############################################# DUAS JANELAS QUE DIVIDEM A IMAGEM EM 2 ####################
                search_top9 = 0
                search_bot9 = 240 
                mask9[0:search_top9, 0:w] = 0 ## 0 - 450 
                mask9[search_bot9:h, 0:w] = 0 ## 470 - 600

                search_top10 = 240 ## 450 (Eu mudei pra 290 então os valores comentados vão ta diferente)
                search_bot10 = 480 ## 470
                mask10[0:search_top10, 0:w] = 0 ## 0 - 450 
                mask10[search_bot10:h, 0:w] = 0 ## 470 - 600

                ##########################################################################################################

                

                ## Janelas para extração os 2 pontos de extremo para calcular a curvatura da trajetória e posteriormente calcular a velocidade linear a ser enviada ao UAV

                dist1 = 10 # Para cálculo da curvatura geral da trajetória

                #Mask 12 : Ponto Superior
                search_top12 = 0 
                search_bot12 = dist1 
                mask12[0:search_top12, 0:w] = 0 
                mask12[search_bot12:h, 0:w] = 0 

                # Mask13 : Ponto inferior
                search_top13 = 480 - dist1 
                search_bot13 = 480 
                mask13[0:search_top13, 0:w] = 0
                mask13[search_bot13:h, 0:w] = 0
                ###


                ### Janelas centrais para o cálculo dos Pontos centrais da imagem (Um no centro e dois nas redondezas)

                dist = 20

                search_top = 230 ## 450 (Eu mudei pra 290 então os valores comentados vão ta diferente)
                search_bot = search_top + dist ## 470
                mask[0:search_top, 0:w] = 0 ## 0 - 450 
                mask[search_bot:h, 0:w] = 0 ## 470 - 600

                search_top1 = search_top - dist ## 430
                search_bot1 = search_top1 + dist ## 450 
                mask1[0:search_top1, 0:w] = 0 ## 0 - 430
                mask1[search_bot1:h, 0:w] = 0 ## 450 - 600
                
                search_top2 = search_bot1 + dist ## 470
                search_bot2 = search_top2 + dist ## 490 
                mask2[0:search_top2, 0:w] = 0 ## 0 - 470
                mask2[search_bot2:h, 0:w] = 0 ## 490 - 600

                ###############################


                ########### Tamanho das janelas variáveis baseadas no tamanho da linha encontrada pela câmera

                janela_superior = int(self.tamanho_linha(mask9,image))
                janela_inferior = int(self.tamanho_linha(mask10,image))
                #janela_superior = 20
                #janela_inferior = 20


                ################## Pontos após a falha #####################

                search_top3 = 0 
                search_bot3 = janela_superior ## 
                mask3[0:search_top3, 0:w] = 0 ## 0 - 470
                mask3[search_bot3:h, 0:w] = 0 ## 490 - 600

                search_top4 = janela_superior ## 470
                search_bot4 = 2*janela_superior ## 490 
                mask4[0:search_top4, 0:w] = 0 ## 0 - 470
                mask4[search_bot4:h, 0:w] = 0 ## 490 - 600

                search_top5 = 2*janela_superior ## 470
                search_bot5 = 3*janela_superior ## 490 
                mask5[0:search_top5, 0:w] = 0 ## 0 - 470
                mask5[search_bot5:h, 0:w] = 0 ## 490 - 600
                ##################################################################

                ######################### Pontos Antes da falha ###################

                search_top6 = 480 - 3*janela_inferior 
                search_bot6 = 480-  2*janela_inferior ## 
                mask6[0:search_top6, 0:w] = 0 ## 0 - 470
                mask6[search_bot6:h, 0:w] = 0 ## 490 - 600

                search_top7 = 480 - 2*janela_inferior ## 470
                search_bot7 = 480 - janela_inferior ## 490 
                mask7[0:search_top7, 0:w] = 0 ## 0 - 470
                mask7[search_bot7:h, 0:w] = 0 ## 490 - 600

                search_top8 = 480 - janela_inferior ## 470
                search_bot8 = 480 ## 490 
                mask8[0:search_top8, 0:w] = 0 ## 0 - 470
                mask8[search_bot8:h, 0:w] = 0 ## 490 - 600
                #######################################################################

                #As máscaras servem para limitar a extração dos centróides em uma determinada faixa de pixels (Faixas de altura h(600 pixels))

                ############################# CV2.MOMENTS EXTRAI A GEOMETRIA DA LINHA REFERENTE A JANELA ESPECIFICADA #####################################
                M13 = cv2.moments(mask13)
                M12 = cv2.moments(mask12) 
                M9 = cv2.moments(mask9)
                M8 = cv2.moments(mask8)
                M7 = cv2.moments(mask7)
                M6 = cv2.moments(mask6)
                M5 = cv2.moments(mask5)
                M4 = cv2.moments(mask4)
                M3 = cv2.moments(mask3)
                #### Ponto do meio
                M2 = cv2.moments(mask2)
                M1 = cv2.moments(mask1)
                M = cv2.moments(mask)
                flag = 1

                ##################################################################################################################################

                if janela_superior < janela_inferior :
                        flag = 1
                else : # janela superior > janela inferior
                        flag = 0

                ##################################################### CÁLCULO DOS CENTRÓIDES E DO ERRO LONGITUDINAL QUE SERVE DE PARÂMETRO DE ENTRADA PARA OS CONTROLES E PONTOS VIZINHOS PARA O CÁLCULO DA CURVATURA DA TRAJETÓRIA

                if M['m00'] > 0 and M1['m00'] > 0 and M2['m00'] > 0  : ## Encontrou o Ponto Principal : Pr

                        cx = int(M['m10']/M['m00'])
                        cy = int(M['m01']/M['m00'])
                        cv2.circle(image, (cx, cy), 5, (0,0,255), -1)
                        self.erro = cx - w/2 # Erro é o ponto do centróide em relação ao centro da câmera (w/2)
                
                
                        cx1 = int(M1['m10']/M1['m00'])
                        cy1 = int(M1['m01']/M1['m00'])
                        cv2.circle(image, (cx1, cy1), 5, (255,0,0), -1)

                        
                        cx2 = int(M2['m10']/M2['m00'])
                        cy2 = int(M2['m01']/M2['m00'])
                        cv2.circle(image, (cx2, cy2), 5, (255,0,0), -1)

                        a = np.array([[cx1,cy1],[cx,cy],[cx2,cy2]]) # Pontos para o cálculo da curvatura 

                elif M3['m00'] > 0 and M7['m00'] > 0 and M8['m00'] > 0 and flag == 1  : ## Caso não encontrou a linha no ponto central e encontrou pontos antes e depois da falha - ENTRAR AQUI

                        cx3 = int(M3['m10']/M3['m00'])
                        cy3 = int(M3['m01']/M3['m00'])
                        cv2.circle(image, (cx3, cy3), 5, (255,0,0), -1)

                        cx7 = int(M7['m10']/M7['m00'])
                        cy7 = int(M7['m01']/M7['m00'])
                        cv2.circle(image, (cx7, cy7), 5, (255,0,0), -1)

                        cx8 = int(M8['m10']/M8['m00'])
                        cy8 = int(M8['m01']/M8['m00'])
                        cv2.circle(image, (cx8, cy8), 5, (255,0,0), -1)

                        
                        x1 = cy3
                        y1 = cx3
                        x2 = cy7
                        y2 = cx7
                        x3 = cy8
                        y3 = cx8
                        x300,x280,x320 = self.get_curva(x1,y1,x2,y2,x3,y3)
                        cv2.circle(image, (x300,240), 5, (0,255,0), -1)
                        cv2.circle(image, (x280,220), 5, (0,255,0), -1)
                        cv2.circle(image, (x320,260), 5, (0,255,0), -1)
                        self.erro = x300 - w/2 # Erro é o ponto do centróide em relação ao centro da câmera (w/2)
                        a = np.array([[x280,220],[x300,240],[x320,260]]) # Pontos para o cálculo da curvatura 

                elif M3['m00'] > 0 and M4['m00'] > 0 and M8['m00'] > 0 and flag == 0  : ## Caso não encontrou a linha no ponto central e encontrou pontos antes e depois da falha - ENTRAR AQUI

                        cx3 = int(M3['m10']/M3['m00'])
                        cy3 = int(M3['m01']/M3['m00'])
                        cv2.circle(image, (cx3, cy3), 5, (255,0,0), -1)

                        cx4 = int(M4['m10']/M4['m00'])
                        cy4 = int(M4['m01']/M4['m00'])
                        cv2.circle(image, (cx4, cy4), 5, (255,0,0), -1)

                        cx8 = int(M8['m10']/M8['m00'])
                        cy8 = int(M8['m01']/M8['m00'])
                        cv2.circle(image, (cx8, cy8), 5, (255,0,0), -1)

                        
                        x1 = cy3
                        y1 = cx3
                        x2 = cy4
                        y2 = cx4
                        x3 = cy8
                        y3 = cx8
                        x300,x280,x320 = self.get_curva(x1,y1,x2,y2,x3,y3)
                        cv2.circle(image, (x300,240), 5, (0,255,0), -1)
                        cv2.circle(image, (x280,220), 5, (0,255,0), -1)
                        cv2.circle(image, (x320,260), 5, (0,255,0), -1)
                        self.erro = x300 - w/2 # Erro é o ponto do centróide em relação ao centro da câmera (w/2)
                        a = np.array([[x280,220],[x300,240],[x320,260]]) # Pontos para o cálculo da curvatura 

                

                elif M3['m00'] > 0 and M4['m00'] > 0 and M5['m00'] > 0  : ## Caso não encontrou a linha no ponto central e encontrou 3 pontos depois da falha - ENTRAR AQUI

                        cx3 = int(M3['m10']/M3['m00'])
                        cy3 = int(M3['m01']/M3['m00'])
                        cv2.circle(image, (cx3, cy3), 5, (255,0,0), -1)

                        cx4 = int(M4['m10']/M4['m00'])
                        cy4 = int(M4['m01']/M4['m00'])
                        cv2.circle(image, (cx4, cy4), 5, (255,0,0), -1)

                        cx5 = int(M5['m10']/M5['m00'])
                        cy5 = int(M5['m01']/M5['m00'])
                        cv2.circle(image, (cx5, cy5), 5, (255,0,0), -1)

                        
                        x1 = cy3
                        y1 = cx3
                        x2 = cy4
                        y2 = cx4
                        x3 = cy5
                        y3 = cx5
                        x300,x280,x320 = self.get_curva(x1,y1,x2,y2,x3,y3)
                        cv2.circle(image, (x300,240), 5, (0,255,0), -1)
                        cv2.circle(image, (x280,220), 5, (0,255,0), -1)
                        cv2.circle(image, (x320,260), 5, (0,255,0), -1)
                        self.erro = x300 - w/2 # Erro é o ponto do centróide em relação ao centro da câmera (w/2)
                        a = np.array([[x280,220],[x300,240],[x320,260]]) # Pontos para o cálculo da curvatura 
                
                elif M6['m00'] > 0 and M7['m00'] > 0 and M8['m00'] > 0  : ## Caso não encontrou a linha no ponto central e encontrou 3 pontos antes da falha - ENTRAR AQUI

                        cx6 = int(M6['m10']/M6['m00'])
                        cy6 = int(M6['m01']/M6['m00'])
                        cv2.circle(image, (cx6, cy6), 5, (255,0,0), -1)

                        cx7 = int(M7['m10']/M7['m00'])
                        cy7 = int(M7['m01']/M7['m00'])
                        cv2.circle(image, (cx7, cy7), 5, (255,0,0), -1)

                        cx8 = int(M8['m10']/M8['m00'])
                        cy8 = int(M8['m01']/M8['m00'])
                        cv2.circle(image, (cx8, cy8), 5, (255,0,0), -1)

                        
                        x1 = cy6
                        y1 = cx6
                        x2 = cy7
                        y2 = cx7
                        x3 = cy8
                        y3 = cx8
                        x300,x280,x320 = self.get_curva(x1,y1,x2,y2,x3,y3)
                        cv2.circle(image, (x300,240), 5, (0,255,0), -1)
                        cv2.circle(image, (x280,220), 5, (0,255,0), -1)
                        cv2.circle(image, (x320,260), 5, (0,255,0), -1)
                        self.erro = x300 - w/2 # Erro é o ponto do centróide em relação ao centro da câmera (w/2)
                        a = np.array([[x280,220],[x300,240],[x320,260]]) # Pontos para o cálculo da curvatura 

                if M8['m00'] > 0 :
                        cx8 = int(M8['m10']/M8['m00'])
                        cy8 = int(M8['m01']/M8['m00'])
                        self.erro_atual = cx8 - w/2
                        #conv = 0.00125*1.5
                        conv = 0.00116 
                        self.ze_atual = (self.erro_atual)*(conv)

                if M9['m00'] > 0 :
                        cx9 = int(M9['m10']/M9['m00'])
                        cy9 = int(M9['m01']/M9['m00'])

                

                ############################################################# CÁLCULO DA VELOCIDADE VARIÁVEL ###################################################################

                Cmin = 0
                #Cmax = 0.0012 #Circuito 1
                #Cmax = 0.01
                Cmax = 0.002 #Circuito 2
                Vmin = 0.01
                Vmax = 0.04
                
                if M['m00'] > 0 and M13['m00'] > 0 : # Existe a curva sem falha do início ao fim da imagem


                        cx13 = int(M13['m10']/M13['m00'])
                        cy13 = int(M13['m01']/M13['m00'])

                        cv2.circle(image, (cx13, cy13), 5, (0,0,255), -1)
                        if M12['m00'] > 0 :
                                cx12 = int(M12['m10']/M12['m00'])
                                cy12 = int(M12['m01']/M12['m00'])
                                cv2.circle(image, (cx12, cy12), 5, (0,0,255), -1)
                                a1 = np.array([[cx12,cy12],[cx,cy],[cx13,cy13]])
                        else : # A curva não está na imagem toda
                                a1 = np.array([[cx9,cy9],[cx,cy],[cx13,cy13]])
                                cv2.circle(image, (cx9, cy9), 5, (0,0,255), -1)


                        dx_dt = np.gradient(a1[:, 0])
                        dy_dt = np.gradient(a1[:, 1])
                        velocity = np.array([ [dx_dt[i], dy_dt[i]] for i in range(dx_dt.size)]) ## Vetor Velocidade

                        ds_dt = np.sqrt(dx_dt * dx_dt + dy_dt * dy_dt) ## Vetor Aceleração

                        tangent = np.array([1/ds_dt] * 2).transpose() * velocity ## Vetor Tangente Unitário

                        tangent_x = tangent[:, 0]
                        tangent_y = tangent[:, 1]

                        deriv_tangent_x = np.gradient(tangent_x)
                        deriv_tangent_y = np.gradient(tangent_y)

                        dT_dt = np.array([ [deriv_tangent_x[i], deriv_tangent_y[i]] for i in range(deriv_tangent_x.size)])

                        length_dT_dt = np.sqrt(deriv_tangent_x * deriv_tangent_x + deriv_tangent_y * deriv_tangent_y)

                        normal = np.array([1/length_dT_dt] * 2).transpose() * dT_dt # Vetor Unitário Normal

                        ## Cálculo das Derivadas segundas
                        d2s_dt2 = np.gradient(ds_dt) 
                        d2x_dt2 = np.gradient(dx_dt)
                        d2y_dt2 = np.gradient(dy_dt)

                        curvature1 = np.abs(d2x_dt2 * dy_dt - dx_dt * d2y_dt2) / (dx_dt * dx_dt + dy_dt * dy_dt)**1.5 ## Cálculo da Curvatura
                        #print(curvature1[1])

                        vlin = ((Vmin - Vmax)*(curvature1[1] - Cmin) + Vmax*(Cmax - Cmin))/(Cmax - Cmin)
                else : 
                        vlin = 0.01

                if vlin < Vmin :
                        vlin = Vmin
                print ("vlin:", vlin)

                ###############################################################################################################################################################################################

                ########################################################################## CÁLCULO DA CURVATURA DA TRAJETÓRIA #################################################################################
                #Cálculo das derivadas Primeiras
                dx_dt = np.gradient(a[:, 0])
                dy_dt = np.gradient(a[:, 1])
                velocity = np.array([ [dx_dt[i], dy_dt[i]] for i in range(dx_dt.size)]) ## Vetor Velocidade
                #print(velocity)
                ds_dt = np.sqrt(dx_dt * dx_dt + dy_dt * dy_dt) ## Vetor Aceleração

                tangent = np.array([1/ds_dt] * 2).transpose() * velocity ## Vetor Tangente Unitário

                tangent_x = tangent[:, 0]
                tangent_y = tangent[:, 1]

                deriv_tangent_x = np.gradient(tangent_x)
                deriv_tangent_y = np.gradient(tangent_y)

                dT_dt = np.array([ [deriv_tangent_x[i], deriv_tangent_y[i]] for i in range(deriv_tangent_x.size)])

                length_dT_dt = np.sqrt(deriv_tangent_x * deriv_tangent_x + deriv_tangent_y * deriv_tangent_y)

                normal = np.array([1/length_dT_dt] * 2).transpose() * dT_dt # Vetor Unitário Normal

                ## Cálculo das Derivadas segundas
                d2s_dt2 = np.gradient(ds_dt) 
                d2x_dt2 = np.gradient(dx_dt)
                d2y_dt2 = np.gradient(dy_dt)

                curvature = np.abs(d2x_dt2 * dy_dt - dx_dt * d2y_dt2) / (dx_dt * dx_dt + dy_dt * dy_dt)**1.5 ## Cálculo da Curvatura
                


                #########################################################################################################################################

                ######################################### CÁLCULO DOS PARÂMETROS QUE SERÃO ENVIADOS PARA O OTIMIZADOR ###################################

                #distancia_linha = np.amin(self.b)*np.cos(45) # A Altura é a distância do depth point multiplicado pela angulação da câmera

                #vel_linear = vlin ##### VELOCIDADE VARIÁVEL
                vel_linear = 0.025 ##### VELOCIDADE LINEAR

                vel_linearx = vel_linear
                vel_lineary = vel_linear
                vel_lineary2 = 0.1-vlin
                ################### CONVERSÃO DE PIXELS PARA METROS #################

                #conv = 0.00125*2 #Conversão de metros (Para 1.5m)
                #conv = 0.00125 #Para 0.7m
                #conv = 0.00125*1.5 #para 1m
                conv = 0.00116 #para 0,93 bebop

                altitude = 1.5 ########### ALTIDUDE A SER SEGUIDA PELO DRONE PARA O SEGUIMENTO DO CIRCUITO

                distH = altitudeZ*np.tan(np.pi/3) # Seguidor do circuito (solo)
                #distH = distancia_linha*np.tan(np.pi/4) #Seguidor de linha de transmissão (linha)

                ############################## ERROS ######################################
                self.ze = (self.erro)*(conv) # Z (metros) é igual ao erro (o quanto o centróide está distante do centro da câmera) multiplicado por um fator de conversão (pixel para metros)
                self.phir = np.arctan2(self.ze,distH) # ThetaR é igual a tangente entre o erro Z e a distância da câmera ao centróide (z) em Rad.

                ##############################################################################

                ## Algoritimo para calcular a média do erro enquanto o drone segue a trajetória (desconsiderar as primeiras amostras)
                delay = 40
                if self.iter > delay :
                        self.soma_z = self.soma_z + abs(self.ze)
                        self.media_z = self.soma_z/(self.iter - delay) 
                        #print(self.media_z)


                self.r = self.r_anterior
                
                ############################################## PID PARA SEGUIMENTO DA LINHA ###########################################
                kp1 = 3.15
                ki1 = 1
                kd1 = 0.0
                
                dt = rospy.get_time() - self.ptime
                #print (dt)
                if dt > 1 :
                        dt = 0.1
                #print (dt)
                self.integral1 += self.ze
                self.integral1 = self.integral1*dt
                self.derivative1 = (self.ze - self.last_erro1) / dt
                self.last_erro1 = self.ze
                
                #uoptimal = -(kp1*self.ze + ki1*self.integral1 + kd1*self.derivative1) # CALCULO DO GANHO PID 

                #self.r_atual = uoptimal #Para o PID

                ################################################################################################

                ####################################### NMPC PARA SEGUIMENTO DA LINHA ####################################################
                self.last_time = rospy.get_time()
                self.otimiza = self.callback_client(roll,pitch,yaw,self.ze,self.phir,self.r,distH,vel_linear) # Manda as variáveis de interesse para o otimizador.
                #time = rospy.get_time() - self.last_time


                #print(self.media_time)

                uoptimal = self.resposta.u ## A variavel de entrada u é recebida pela variável resposta.u .
                tempo = self.resposta.v
                self.soma_time = self.soma_time + tempo
                self.media_time = self.soma_time/self.iter
                #print(self.media_time)
                #print(tempo)
                print("Erro:", self.ze)
                self.r_atual = (uoptimal*np.cos(self.phir) + curvature[1]*((np.cos(pitch)*np.cos(yaw)*vel_linearx)+(np.sin(roll)*np.sin(pitch)*np.cos(yaw)- np.cos(roll)*np.sin(yaw))*vel_lineary) / ((np.cos(self.phir) - curvature[1]*self.ze)*(np.cos(roll)/np.cos(pitch)))) # Cálculo do omega que será aplicado ao robô

                #######################################      O OBJETO TWISTS RECEBE AS VELOCIDADES LINEARES E ANGULARES QUE SERÃO ENVIADAS AO DRONE ######################################################
                
                #if(xr>=0):
                    #self.twist.linear.x = vel_linear # Velocidade linear em X
                 #   self.twist.linear.x = 0 # Velocidade linear em X
                if(self.ze<=-0.15):
                    self.twist.linear.y = vel_linear*(abs(self.ze)/0.35)# Velocidade linear em Y
                    #self.twist.linear.y = 0 # Velocidade linear em X
                    self.twist.linear.x = vel_linear*0.5
                    #self.twist.linear.x =0
                    self.twist.angular.z = 0.01*self.r_atual
                if(self.ze>=0.15):
                    self.twist.linear.y = (-1)*vel_linear*(abs(self.ze)/0.35) # Velocidade linear em Y
                    #self.twist.linear.y = 0 # Velocidade linear em X
                    self.twist.linear.x = vel_linear*0.5
                    #print('passou 2')
                    #self.twist.linear.x =0
                    self.twist.angular.z = 0.01*self.r_atual
                #if(abs(self.ze) < 0.2):
                if(0.15>self.ze>-0.15):
                    self.twist.linear.y=0
                    self.twist.linear.x = vel_linear
                    self.twist.angular.z = 0.1*self.r_atual #0.09

                    #print('passou 3')
                P = 0.5#testes reais 0.2 0.0005
                I = 0.03
                D = 0.000
                #self.twist.angular.z = self.pid(self.r_atual, P, I, D,dt) # Para o PID e NMPC puro




 
                
                ##########################################################################################################################################################################################

                '''
                # Parte do código para dar meia volta caso não encontre a linha
                if M['m00'] > 0 and M1['m00'] > 0 and M2['m00'] > 0  :

                        self.twistS.twist.linear.x = vel_linear*np.cos(yaw)
                        self.twistS.twist.linear.y = vel_linear*np.sin(yaw)

                        self.twistS.twist.angular.z = self.r_atual # Para o PID e NMPC puro
                        print ('Encontrou a linha - Seguindo-a')
                
                else :
                        self.twistS.twist.linear.x = 0
                        self.twistS.twist.linear.y = 0
                        self.twistS.twist.angular.z = 1
                        print ('Encontrou uma falha - Fazendo manobra de Meia volta')
                '''

                #self.twistS.twist.linear.x = 0
                #self.twistS.twist.linear.y = 0
                #print (self.r_atual)
                #dt = rospy.get_time() - self.ptime
                #self.twistS.twist.angular.z = self.pid(self.r_atual,2.5,0.05,0,0.1) ## Circuito 1
                #self.twistS.twist.angular.z = self.pid(self.r_atual,3,0.05,0,0.1) ## Circuito 2 (teste)
                #self.twistS.twist.angular.z = self.pid(self.r_atual,0.8,0.05,0.005,dt) ## Linha de transmissão


                self.iter = self.iter + 1
                self.r_anterior = self.r_atual   
                #print(self.ze)
                
                ############################################################### CONTROLE DE ALTITUDE ##########################################################

                ########################## PARA O CASO DE SEGUIMENTO DA LINHA DE TRANSMISSÃO ##################################################################
                setpoint_linha = 1.5
                #d_altitude = setpoint_linha - distancia_linha
                d_altitude = altitude - altitudeZ

                #self.twist.linear.z = self.pidZ(d_altitude, 0.5, 0.001, 0.1) ## PID para manter o drone a uma altitude constante
                #self.twistS.twist.linear.z = self.pidZ(d_altitude, 4, 0.01,0.0,dt) ## PID para manter o drone a uma distancia da linha constante
                
 

                #################################################################################################################################################

                ########################################################## AS MENSAGENS SÃO ENVIADAS AO DRONE PELO COMANDO CMD_VEL ##############################
                if(self.twist.angular.z > 0.1): #0.04 funfando 
                    self.twist.angular.z = 0.1
                if(self.twist.angular.z < -0.1):
                    self.twist.angular.z = -0.1

                if(self.twist.linear.y > 0.025): #0.025 funfando 
                    self.twist.linear.y = 0.025
                if(self.twist.linear.y < -0.025):
                    self.twist.linear.y = -0.025

                if(xm == []):
                    self.cmd_vel_pub.publish(self.twist)
                    
                else:
                    self.flagalt = 1
                   
                    self.twist.linear.x = 0
                    self.twist.linear.y = 0
                    self.twist.angular.x = 0
                    self.twist.angular.y = 0
                    self.twist.angular.z = 0
                    self.cmd_vel_pub.publish(self.twist)
                #################################################################################################################################################
                self.ptime = rospy.get_time()
                #print(self.twist.angular.z)
                #print(self.twist.linear.x)
                #print(self.twist.linear.y)  
                #print(curvature[1]) 

                ################################### IMPRESSÃO DOS DADOS EM ARQUIVOS DE TEXTO ####################################################
                #with open("/home/gapc/catkin_ws/src/nmpc_robo/src/scripts/upot.txt", "a") as output:
                 #       output.write("%s \n" % altitudeZ)
                #with open("/home/gapc/catkin_ws/src/nmpc_robo/src/scripts/phir.txt", "a") as output:
                #        output.write("%s \n" % self.phir)
                #with open("/home/gapc/catkin_ws/src/nmpc_robo/src/scripts/ze.txt", "a") as output:
                #        output.write("%s \n" % distancia_linha)
                ################################################################################################################################


                #print (len(mask9[1]))
                #cv2.imshow("window", image)
                cv2.imshow("window1",mask9)
                #cv2.imshow("window1",mask11)
                #cv2.imshow("window",mask)

                cv2.waitKey(3)



rate = rospy.Rate(5)
follower = Follower()
rospy.spin() #Comando que mantem o no do ROS em loop ate ser cancelado

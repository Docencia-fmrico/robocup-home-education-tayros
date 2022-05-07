[![Open in Visual Studio Code](https://classroom.github.com/assets/open-in-vscode-c66648af7eb3fe8bc4f294546bfd86ef473780cde1dea487d3c4ff354943c9ae.svg)](https://classroom.github.com/online_ide?assignment_repo_id=7696783&assignment_repo_type=AssignmentRepo)

# RoboCup

<div align="center">
<img width=600px src="https://github.com/Docencia-fmrico/robocup-home-education-tayros/blob/readme/resources/figures/RoboTay.png" alt="explode"></a> 
</div>

<h3 align="center"> RoboCup </h3>

<div align="center">
<img width=100px src="https://img.shields.io/badge/status-finished-brightgreen" alt="explode"></a>
<img width=100px src="https://img.shields.io/badge/license-Apache-orange" alt="explode"></a>
<img width=90px src="https://img.shields.io/badge/team-TayRos-yellow" alt="explode"></a>
</div>


## Table of Contents
- [Table of Contents](#table-of-contents)
- [Task1 - Carry my luggage](#task1-carry-my-luggage)
- [Task2 - Find my mates](#task2-find-my-mates)
- [Team](#team)
- [Licencia](#licencia)

## Task1: Carry my luggage 
                                                                                  
### Perception
                                                                                   
#### Pointing Detector

To detect which suitcase the referee points to, we use the **rescaling of the bounding box**. The BT tells the referee to keep still and then activates the **PointingDetector node**, which captures the sizes of the bbx in the initial state and with the successive rescalings calculates the differences to know where he is pointing. 

**Preview:**
<div align="center">
<img width=600px src="https://github.com/Docencia-fmrico/robocup-home-education-tayros/blob/readme/resources/figures/tayvision_pointing.gif" alt="explode"></a> 
</div>

[Full video](https://urjc-my.sharepoint.com/:v:/g/personal/i_porras_2020_alumnos_urjc_es/EVwnWIlAgrJKg8ZxPvrFd6MB7xdHOAQ-IWKf1sYGPvelQQ?e=Td46gQ)





#### Person Filter
To remember the referee and distinguish him/her from the rest of the people we implemented a double filter. The main idea is to detect the bounding boxes of people through darknet ros and then filter these bbx to publish in a new topic **"/bbx_filtered/bounding_boxes/person"** the bbx of the filtered person (in this case the referee).

This  algorithm has two phases:

**1: Motion estimation**: This node will be processing images with a frequency of 20 Hz so we can estimate that the person will not be able to move from one image to the next more than a **small angle and distance**. With this **we filter most of the bbx** and we **avoid that the people that are in the background are analyzed** avoiding that expense of resources.


**2: Color filter**: When executing the node this **divides the bbx of the person to filter in segments and calculates the color average** of each one of these segments and stores them in a **buffer**. All the bbx that have exceeded the position estimation are compared with the values of the buffer and **passed through a probability function** which will return the percentage of similarity and if this is superior to the established threshold it is published as the filtered person and u**pdates the lower value of similarity of the buffer for this last one**.

<br>

**DIAGRAM:**
<div align="center">
<img width=600px src="https://github.com/Docencia-fmrico/robocup-home-education-tayros/blob/readme/resources/figures/1-%20Filtro%20Movimiento.png" alt="explode"></a> 
</div>

<br><br>

It also has several ways of recovery in case of changes in brightness and loss of the person.

For this tracking we also implemented the "BbxTo3D" node which received a bbx, in this case the filtered one, and taking advantage of the camera characteristics obtained from the "/camera/depth/camera_info" topic and the "image_geometry" package, it calculated the 3D point of this bbx in order to navigate to the referee.


<br><br>


**PREVIEW:**
<div align="center">
<img width=600px src="https://github.com/Docencia-fmrico/robocup-home-education-tayros/blob/readme/resources/figures/tayvision_filtertwo.gif" alt="explode"></a> 
</div>

[Full video](https://urjc-my.sharepoint.com/:v:/g/personal/i_porras_2020_alumnos_urjc_es/ERh2wiJXTRJJlCKO51SO13YB9ZKmhjeo9cCi1EREfwXstQ?e=N9E1Kt)


                                                                                                                                     
                                                                                   
                                                                                   
## Task2: Find my mates  

### Perception

#### Object Detector
To detect the object that the person is carrying we use thedarknet ros detection and the images from the depth camera to keep the detected object that is closest to the person both in distance and angle.


We also store the objects we are interested in identifying in an array to avoid fake detections.

**PREVIEW:**
<div align="center">
<img width=600px src="https://github.com/Docencia-fmrico/robocup-home-education-tayros/blob/readme/resources/figures/tayvision_object_final.gif" alt="explode"></a> 
</div>


#### Shirt Color Detection

To detect the color of the person's T-shirt, we carry out several phases:
1. **Resize of the bounding box**: The resulting bbx will be on the chest of the person which will allow us to analyze only the part we are interested in.

2. Calculation of the **average color of the pixels** of the new bbx (as in the filter of the first test).

3. Compare the results of these segments with our list of colors [Black, White, Red, Pink, Light Blue, Yellow, Green, Blue].

Each time a color is detected it increments it's **counter** until one of the colors exceeds the publication threshold.

**PREVIEW:**
<div align="center">
<img width=600px src="https://github.com/Docencia-fmrico/robocup-home-education-tayros/blob/readme/resources/figures/tayvision_color2_final.gif" alt="explode"></a> 
</div>




                                                                                  

## Team
<div align="center">
<img width=600px src="https://github.com/Docencia-fmrico/bump-and-go-with-fsm-tayros/blob/main/resources/grupo.jpg?raw=true"  alt="explode"></a>
</div>
<h5 align="center">TayRos 2022</h5
  
- [Saul Navajas](https://github.com/SaulN99)
- [Adrian Madinabeitia](https://github.com/madport)
- [Ivan Porras](https://github.com/porrasp8)

## Licencia 
<a rel="license" href="https://www.apache.org/licenses/LICENSE-2.0"><img alt="Apache License" style="border-width:0" src="https://www.apache.org/img/asf-estd-1999-logo.jpg" /></a><br/>(TayRos) </a><br/>This work is licensed under a <a rel="license" href="https://www.apache.org/licenses/LICENSE-2.0">Apache license 2.0

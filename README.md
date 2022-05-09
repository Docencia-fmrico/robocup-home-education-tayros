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

### Dialog
We use **DialogFlow** to manage the robot dialog. Using the **intents** we manage all the different tasks of the dialog-part, as we show below.
The dialog is divided in the listening part, as well as the speech part.

The speech is implemented in the different methods and callbacks of **dialog_cbs**

#### Listening node
The dialog part has been implemented using, basically , a simple node (**dialog_cbs_node**) that listens in a ros ok loop.

-----------------------------------------------------------------------
Snippet(dialog_cbs_node):
``` cpp
int main(int argc, char** argv)
{
  ros::init(argc, argv, "dialog_cbs_node");

  gb_dialog::DialogManager forwarder;
  ros::Duration(1, 0).sleep();

  while(ros::ok())
  {
    forwarder.listen();

    ros::spinOnce();
  }
  return 0;
}
```
-----------------------------------------------------------------------
#### Start phase
In order to use the start-voice-command and start following the person after the luggage is selected/pointed, we created a DialogFlow intent that allowed us to detect voice-orders like "**start**" or "**lets go**". This same intent is also used in order to start the navigation part of **Carry My Luggage**.

Here's a little example of how it works:

[Watch Video](https://urjc-my.sharepoint.com/:v:/g/personal/s_navajas_2020_alumnos_urjc_es/EfXy9xZWqBdDkQvSHqq4n7YBEB_ZMpmM5TfZ10RLvdQZJg?e=Ifaj1i)

#### Point Luggage Backup
If, by any case, the pointed luggage is not detected using the bouding box, we ask the operator to say which luggage he wants to carry. The robot is able to detect:

**- "The one/bag of your right/left"**

**- "The one/bag of my right/left"**

**- "right/left"**

-----------------------------------------------------------------------
Snippet(pointBagCB):
``` cpp
void
DialogManager::pointBagDialogCB(dialogflow_ros_msgs::DialogflowResult result)
{
  ROS_INFO("[TAY_DIALOG] pointBagDialogCB:");
  pointedBag_ = result.fulfillment_text;
  questionAsked_ = true;
  pointBag(2);
}
```
-----------------------------------------------------------------------

Here's an example of how it works:

[Watch Video](https://urjc-my.sharepoint.com/:v:/g/personal/s_navajas_2020_alumnos_urjc_es/ERwDE949KWFIsIsD1d_kJAEB8Ia2zTsl5uB1exj90tpOeQ?e=dgozmg)

#### Car reached
The operator is able to say "**stop**" to the robot in order to tell him that he has arrived to the car.

Here's an example:

[Watch Video](https://urjc-my.sharepoint.com/:v:/g/personal/s_navajas_2020_alumnos_urjc_es/EbAaTYpAVj1Ciy6LYaGm734BkECcbkIjp5PRZkO88QG04w?e=psTG9n)
                                                                                                                                     

### Logic and BT's

The logic is divided in three behavior trees:

#### Init sequence
The first movements of the robot, first in welcome_human the robots waits a intstruction to start. Then, the robot go to the referee and then call the node Localize_suitcase.

<div align="center">
<img width=600px src="https://github.com/Docencia-fmrico/robocup-home-education-tayros/blob/readme/resources/figures/tayjarvis_init_sequence_BT.png?raw=true" alt="explode"></a> 
</div>

Localize suitcase not only reports the activation of the pointing_detector_node but also restarts the bbx for more precision. Then pointing_detector_node returns the side that the referee pointed and turns the robot.

#### Follow
We managed two ways to follow a person. One was follow the person with navigation, the node needed to get the TF of the person, converse it into a Goal, calculate the route to the person with the service /move_base/make_plan and take the goal neareast to the person. The problem was that sometimes the robot loses the person, and that you have to point out the orientation of the robot, sometimes the orientation was wrong and the robot started to turn when it arrived to the point. 

Finally we recycled the follow person code aun used it to follow the person.
<div align="center">
<img width=600px src="https://github.com/Docencia-fmrico/robocup-home-education-tayros/blob/readme/resources/figures/tayjarvis_follow_bt.png?raw=true" alt="explode"></a> 
</div>

Conexions between nodes and BT:
<div align="center">
<img width=600px src="https://github.com/Docencia-fmrico/robocup-home-education-tayros/blob/readme/resources/figures/tayjarvis_gtaphic.png?raw=true" alt="explode"></a> 
</div>

##### Service subscriptor
-----------------------------------------------------------------------
Snippet(pointBagCB):
``` cpp
void
Set_route::makePlan()
{
    nav_msgs::GetPlan new_goal;
    move_base_msgs::MoveBaseGoal goal;
    fillPath(new_goal.request);

    if (route_client_.call(new_goal))
    {
        long index = size(new_goal.response.plan.poses);
        if (index != 0)
        {
            int array_pos = index-1-(index-1)*FACTOR;

            ROS_INFO("Longitud array = %d, Array_pos = %d\n", index, array_pos);
            goal.target_pose.pose.position.x = new_goal.response.plan.poses[array_pos].pose.position.x;
            goal.target_pose.pose.position.y = new_goal.response.plan.poses[array_pos].pose.position.y;
            goal.target_pose.pose.orientation.x = new_goal.response.plan.poses[array_pos].pose.orientation.x;
            goal.target_pose.pose.orientation.y = new_goal.response.plan.poses[array_pos].pose.orientation.y;
            goal.target_pose.pose.orientation.z = new_goal.response.plan.poses[array_pos].pose.orientation.z;
            goal.target_pose.pose.orientation.w = new_goal.response.plan.poses[array_pos].pose.orientation.w;

            if (msg_recived_)
            {
                calculated_pos_pub_.publish(goal);
                msg_recived_ = false;
            }
        }
    }
    else
    {
        ROS_ERROR("Service failed");
    }
}
```
-----------------------------------------------------------------------

### Back_home
This BT is only one node, but we implementd it to improve the modularity of the program.
<div align="center">
<img width=300px src="https://github.com/Docencia-fmrico/robocup-home-education-tayros/blob/readme/resources/figures/tayjarvis_back_home_bt.png?raw=true"></a> 
</div>

                                                                                   
## Task2: Find my mates  

### Perception

#### Object Detector
To detect the object that the person is carrying we use thedarknet ros detection and the images from the depth camera to keep the detected object that is closest to the person both in distance and angle.


We also store the objects we are interested in identifying in an array to avoid fake detections.

**PREVIEW:**
<div align="center">
<img width=600px src="https://github.com/Docencia-fmrico/robocup-home-education-tayros/blob/readme/resources/figures/tayvision_object_final.gif" alt="explode"></a> 
</div>

[Full video](https://urjc-my.sharepoint.com/:v:/g/personal/i_porras_2020_alumnos_urjc_es/EbIAokzyXWZInRZJoARuCtYBl6tYq3upsSsw0M7zFiFrWg?e=E5G9vc)


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

[Red test video](https://urjc-my.sharepoint.com/:v:/g/personal/i_porras_2020_alumnos_urjc_es/ETQMMRqq01dNh-i0XtSzynQBU32ZuxIj9znTBoqM59iPOQ?e=lqTsdS)
<br>
[Black test video](https://urjc-my.sharepoint.com/:v:/g/personal/i_porras_2020_alumnos_urjc_es/ERk_PobuXgxNpW7QbYKhCrcBevCizvQp2AiF1eU8YG_l-w?e=80wOPM)



### Person Localization

To detect people and analyze the area where they are we use the **PersonLocalizator** node. This one is in charge of analyzing the detected people, **obtaining its 3D point** (leaning on the BbxsTo3D node), **the zone** in which this one is and storing each one of the people in a **buffer** to later publish these values to the navigation and to the node in charge of taking the information of each person.

When this node finishes analyzing the subject, it will return a feedback through the topic **"/tayros/person/feedback"** so that it can change to the state of that person and stop publishing it.

The published message is a custom message contained in the **taymsgs**(person_info.msg) package.

**person_info.msg:**
``` cpp
move_base_msgs/MoveBaseGoal position

int32 id
int32 zone
```


### Dialog
#### Ask for name
In **Find My Mates Task**, the robot ask to each person his name. The robot is able to catch **any name the person says**. The person's name is published in a topic to allow the take_person_info_node to manage the information.

-----------------------------------------------------------------------
Snippet(askForNameCB):
``` cpp
  if(name_restart_){
    personName_ = "none";
  }
  if(activation_){  
    ROS_INFO("[TAY_DIALOG] askForNameCB:");
    personName_ = result.fulfillment_text;
    questionAsked_ = true;
    std_msgs::String name;
    name.data = personName_;
    name_pub_.publish(name);
    std::cout << "-------------------------------------------------" << std::endl;
    std::cout << "[TAY_DIALOG] person name is: " << personName_ << std::endl;
    std::cout << "-------------------------------------------------" << std::endl;
  }
```
-----------------------------------------------------------------------
Here's an example of how it works:

[Watch Video](https://urjc-my.sharepoint.com/:v:/g/personal/s_navajas_2020_alumnos_urjc_es/ETFEzV_zr01FjGr57YY7O0QBEx9jQhGVmbUjDpZmj8KA-A?e=dgkZXz)

### Take person info
Once the robot is in front of the person, the **take_person_info_node** proceeds to take and manage all the information he receives from the perception and dialog. In order to do it, we use **a buffer and a custom struct** with the person info:

-----------------------------------------------------------------------
Snippet(Take_person_info):
``` cpp
enum{
    PERSON_BUFFER = 4,
};

typedef struct
{
    std::string name;
    std::string colorShirt;
    std::string object;
    int zone;
    int id;
    move_base_msgs::MoveBaseGoal goal;
}t_personInfo;
```
-----------------------------------------------------------------------

There's a number of different subscribers that gets the information of each person that are published in different topics by the corresponding dialog and perception nodes, and in each callback the information is store in the struct.
**We also check if the person that is in front of the robot has been already studied:**

-----------------------------------------------------------------------
Snippet(Take_person_info):
``` cpp
bool
PersonInfo::is_id_studied(int id){
    for(int i = 0; i < PERSON_BUFFER; i++){
        if(id_studied[i] == id){
            return true;
        }
    }
    return false;
}

...

void
PersonInfo::callback_person_info(const taymsgs::person_info::ConstPtr& person)
{
    if(! person_taked_)
    {
        if(!is_id_studied(person->id)){
            current_person_.id = person->id;
            current_person_.zone = person->zone;
            current_person_.goal = person->position;
            person_taked_ = true;
        }
    }
}
```
-----------------------------------------------------------------------

Once the person has been studied, we send the info to the Behaviour Tree and, if the BT indicates that the info has been commented, we start again studying the information of the next person.          

### Logic and BT's
We have two BT's, in the node wich runs the BT's we have a loop that doesn't start the execution of the program since the user doesn't say lets go or start.

#### Find_mate
<div align="center">
<img width=600px src="https://github.com/Docencia-fmrico/robocup-home-education-tayros/blob/readme/resources/figures/tayfinder_bt_find_mate.png?raw=true"  alt="explode"></a>
</div>

First, the robot moves to a position, then it starts executing bump and go, the other option was navigate to the point of each person. When the robots arrives to the first point. The robot starts searching the person. When the person is found the robot moves to him and starts getting his data.

<div align="center">
<img width=600px src="https://github.com/Docencia-fmrico/robocup-home-education-tayros/blob/readme/resources/figures/tayfinder_comunications.png?raw=true"  alt="explode"></a>
</div>
Get mate data reports that the person localization is reached, the node take_person_info takes all the data and then returns it to get_mate_data

#### Back_home
<div align="center">
<img width=600px src="https://github.com/Docencia-fmrico/robocup-home-education-tayros/blob/readme/resources/figures/tay_finder_back_home_BT.png?raw=true"  alt="explode"></a>
</div>
This back home ends when the robot moves to the refree position and comunicates him the data reccived with the custon msg person_data

**person_data.msg:**
``` cpp
string name
string colorShirt
string object

int32 zone
int32 id

move_base_msgs/MoveBaseGoal goal
```

[Person data comunication](https://urjc-my.sharepoint.com/:v:/g/personal/a_madinabeitia_2020_alumnos_urjc_es/EW2uSR1eOq5Eu0Oz9HqLOTgBxQtxIWvI7Eu6-TtE_0q1wQ?e=5m2BeD)

## Team
<div align="center">
<img width=400px src="https://github.com/Docencia-fmrico/bump-and-go-with-fsm-tayros/blob/main/resources/grupo.jpg?raw=true"  alt="explode"></a>
</div>
<h5 align="center">TayRos 2022</h5
  
- [Saul Navajas](https://github.com/SaulN99)
- [Adrian Madinabeitia](https://github.com/madport)
- [Ivan Porras](https://github.com/porrasp8)

## Licencia 
<a rel="license" href="https://www.apache.org/licenses/LICENSE-2.0"><img alt="Apache License" style="border-width:0" src="https://www.apache.org/img/asf-estd-1999-logo.jpg" /></a><br/>(TayRos) </a><br/>This work is licensed under a <a rel="license" href="https://www.apache.org/licenses/LICENSE-2.0">Apache license 2.0

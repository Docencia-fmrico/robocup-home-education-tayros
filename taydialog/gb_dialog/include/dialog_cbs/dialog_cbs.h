// Copyright 2022 TayRos
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef DIALOG_FSM_DIALOG_FSM_H
#define DIALOG_FSM_DIALOG_FSM_H


#include <gb_dialog/DialogInterface.h>
#include <string>

namespace ph = std::placeholders;

namespace gb_dialog
{
class DialogManager: public DialogInterface
{
public:
    DialogManager();

    void noIntentCB(dialogflow_ros_msgs::DialogflowResult result);
    void welcomeIntentCB(dialogflow_ros_msgs::DialogflowResult result);

    void welcomeHumanCML();
    void welcomeHumanFMM();
    void pointBag(int listenFlag);
    void startNav();
    void movementIndications();
    void askForName();
    void askForNameCB(dialogflow_ros_msgs::DialogflowResult result);

    void pointBagDialogCB(dialogflow_ros_msgs::DialogflowResult result);
    void startNavCB(dialogflow_ros_msgs::DialogflowResult result);
    void carReachedCB(dialogflow_ros_msgs::DialogflowResult result);

    void end();

    void gotLostCB();

    std::string isCarReached();
    std::string getPointedBag();

private:
    ros::NodeHandle nh_;

    std::string pointedBag_;
    std::string readyToMove_;
    std::string carReached_;
    std::string personName_;

    bool questionAsked_;

};
};  // namespace gb_dialog


#endif  // DIALOG_FSM_DIALOG_FSM_H
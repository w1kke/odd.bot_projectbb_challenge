# Odd.bot/ProjectBB Coding Challenge

## Motivation:
The reason why we do this coding challenge is to show you and us how the future work will be organised and which level of autonomy is required to work in the thesis.
Additionally we want to make sure that we can communicate with you and you can communicate with us.
As the last point we want to make sure that you are capable of creating a solution to a problem we have to solve and like your coding style. But most of all the use of the tools is important in this challenge.

There will be two challenges and the difficulty will probably not be the same - but both lead to different thesis topics. The first is the Computer Vision challenge for a vision based thesis, the second is the Reinforcement Learning challenge for an actuator based thesis.

In general, if you struggle with the instructions, tools or the task, please feel free to contact me.

Sending in the solution means that you have to add me to your (private) GitHub repository and I will then check out the code. My username on GitHub is w1kke - you can add me right away and then I can check your code while you work on it. In the mail you receive there will be a repository that functions as the start of your challenge and you have to fork it to your own (private) repository. We will work on Docker images to make sure that the execution of the code does not rely on local configurations and can work on your own machine, AWS VMs and my machine as well.

I will also provide you with access to a virtual machine (VM) on Amazon Web Services so you have access to a GPU instance - please see the mail.

## Computer Vision challenge:
This challenge involves training a DNN to recognize cigarette butts.
We follow this blog article where the author created an artificial set of cigarette butt images and already explains that he trained a DNN to recognize them:
https://medium.com/@aktwelve/training-an-ai-to-recognize-cigarette-butts-5cff9e11c0a7
The data is available here:
http://www.immersivelimit.com/datasets/cigarette-butts
Please train the cigarette butts as a new class in YOLOv2 for Tensorflow.

## Reinforcement Learning challenge:
This challenges involves setting up an OpenAI gym environment and getting it to train to pick up an item in this environment - this task is open ended as getting this to run and succeed is already quite a time consuming effort. The further you get the better.
This a blog post about the setup in the OpenAI gym:
https://openai.com/blog/ingredients-for-robotics-research/

And this is the corresponding gym environment:
https://gym.openai.com/envs/FetchPickAndPlace-v0/


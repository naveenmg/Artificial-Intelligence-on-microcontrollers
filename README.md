# Artificial Intelligence on microcontrollers

## ABSTRACT

The goal of this project is to research and test out the applications of artificial intelligence on microcontrollers.
The multiple stages of capturing and processing data, determining the necessary neural network architecture, training the model, converting the model into optimised code and finally to run the code to process the data is dealt with in depth.
The STM32Cube.AI package and micro Tensor are the two alternatives explored and compared in this project.


## SOFTWARES 

### STM32CubeMX with X-CUBE-AI 

-	STM32CubeMX is a graphical tool that allows a very easy configuration of STM32 microcontrollers and microprocessors.
X-CUBE-AI is an STM32Cube Expansion Package part of the STM32Cube.
AI ecosystem and extending STM32CubeMX capabilities with automatic conversion of pre-trained Neural Network and integration of generated optimized library into the user's project.

### Keil

Keil MicroVision is a free software which solves many of the pain points for an embedded program developer.
This software is an integrated development environment (IDE).

### MicroTensor

-	uTensor is an extremely light-weight machine learning inference framework built on Mbed and Tensorflow. 
A model is constructed and trained in TensorFlow. uTensor takes the model and produces a .cpp and .hpp file.

## STEP 1: CAPTURE AND PROCESS DATA

Datasets can be created by capturing multiple thousands of data and then using them as reference for learning.
There are multiple prepared datasets available online such as the MNIST Dataset, database of handwritten digits, and Actitracker dataset, motion tracking dataset released by Wireless Sensor Data Mining (WISDM) lab which have been used for this project.

## AI

The neural network model can be created on Keras running on top of Tensorflow 2.0.
Since it does not run with the latest versions of Python, a specific version of Python can be chosen on a virtual environment such as Anaconda.
Further editing on the model or architecture can be done on Jupyter Notebook to provide a better overview.
Tensorflow 2.0- TensorFlow is an end-to-end open source platform for machine learning.
Keras- The Python Deep Learning library. Keras is a high-level neural networks API, written in Python and capable of running on top of TensorFlow, CNTK, or Theano.
Anaconda- Anaconda is a free and open-source distribution of the Python and R programming languages for scientific computing. 
Juypter Notebook- The Jupyter Notebook is an open-source web application that allows you to create and share documents that contain live code, equations, visualizations and explanatory text.

## STEP 2: COLLECT DATA AND ARCHITECT A NEURAL TOPOLOGY

Design a neural network topology, in this case I have worked mainly with the convoluted neural network topology (CNN) using 80 percent of the data for training and the remaining for testing. 
The model can be created on various other topologies such as a recurrent neural network (RNN).

## NEURAL NETWORK

Building a normal neural network model with CNN topology with the help of the Kaggle dataset, images of various cats and dogs, was vital in understanding the creation and working with a neural network. 
The basic neural network built based on CNN determines whether any given image contains a cat or a dog.

## STEP 3: TRAIN YOUR NEURAL NETWORK MODEL

Before converting the model into C++ code, the model must be trained. 
The model can be trained by validating the model on the target after making sure that it can be validated on the desktop first on the STM32CubeMX application.
The Stm32Cube AI package has a compression algorithm that reduces the size of the model without affecting its functionality. 
If using microTensor the model must be trained instead on Jupyter Notebook.

## STEP 4: CONVERT YOUR NEURAL NETWORK MODEL

Once all the above-mentioned steps are complete the model can be converted into C++ code.
The code is then accessible through software such as Keil. 
The C++ code can then be further edited to meet the requirements. 

## STEP 5: RUN CODE TO PROCESS DATA

The code can be run directly from keil onto the available boards and the neural network model is now readily available within the microcontroller. 
When working with microTensor a mbed OS along with additional main c++ file is necessary to run the converted code along with all the mbed utilities. 

The major difference between the STM32Cube AI package and the microTensor is that the Cube AI package runs on STM32CubeMX while the microTensor is compatible with mbed.
The Cube AI package also has a compression facility which reduces the size of the file. 

## APPLICATIONS

### •	Human Activity Recognition Artificial Intelligence (HAR AI) 
### •	Handwritten digit recognition 

## MicroTensor

### uTensor with Mbed and TensorFlow
 
### Requirements

•	Mbed Command Line Tool (Mbed-cli)
•	uTensor-cli (graph to source compiler)
•	TensorFlow (Comes with uTensor-cli)
•	Mbed board (or, with Mbed Simulator)
•	CoolTerm (For serial communication)

### Steps

1.	CNN model in python or Jupyter Notebook
2.	Convert to Graph (. pb)
3.	Auto generate c++ model with utensor-cli
4.	Compile file on mbed board






# Descibe Objects and Relations in Image

Generating image descriptions has several difficault sub tasks
including recognition of objects and their relations, and 
grounding words in their relevant features.

The goal of this tutorial is to examine some of these difficulties with
generating descriptions	for objects and	their relations in images.

In this task you work with an online demo that generates word sequences and 
also reports how each word in connected to feature representations.

You need to specify an image, and two objects in the image, 
then system proposes word sequence which refers to the objects and their relations.

The neural network model behind this system is using a pretrained convolutional neural networks to extraxt visual features and
a recurrent neural language model to learn contextual embeddings. 
We used sampling with beam search to generate these descriptions. 

Follow this link: [demo.html](demo.html)
 

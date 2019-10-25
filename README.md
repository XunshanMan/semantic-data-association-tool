# Introduction
For the semantic object-level SLAM, we need to solve the data associations of the bounding boxes generated from object detector (eg. Yolo) between frames.

This project aims at building a convenient tool for annotating data association. The User Interface is built upon Qt. 

# Update: 3D annotation module
3D annotation module is part of the automatic annotation. We get the 3D position of the objects and use their 2d projections to help us annotate the 2d bbox.
In this way, only several global 3d objects need annotated, instead of all the 2d image frames.

![img](./.md/3d-1.png)

![img](./.md/3d-2.png)

# Example
Build it.
Run the excutable binary file.

It's friendly for annotating. Just like this:
![img](./.md/p1.png)


# Change by yourself
Load it in Qt Creator from .pro file. Then play it as you like.

# TODO
[ ] Automatic data association: 
It's too much for human annotation alone. Let machine help you.

[ ] Visualization: Show the asssociation better with text, rectangle and something else on the images.

Welcome to join me!

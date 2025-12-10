---
sidebar_position: 1
---

# Chapter 1: ROS 2 Fundamentals - Nodes, Topics, Services

This chapter introduces you to the core concepts of ROS 2: Nodes, Topics, and Services. These form the foundation of all ROS 2 systems and are essential for understanding how robotic components communicate with each other.

## Learning Objectives

After completing this chapter, you will be able to:

- Explain what ROS 2 Nodes are and how they function
- Understand the publish-subscribe communication pattern using Topics
- Implement simple Services for request-response communication
- Create and run basic ROS 2 nodes with Python
- Debug common communication issues in ROS 2 systems

## Sections

This chapter covers:

1. [Understanding ROS 2 Nodes](./nodes) - The basic building blocks of ROS 2 systems
2. [Topics and Message Passing](./topics) - The publish-subscribe communication pattern
3. [Services and Request-Response](./services) - Synchronous communication for specific requests

## Prerequisites

- Basic Python programming knowledge
- ROS 2 Humble Hawksbill or Iron Irwini installed
- Understanding of the module introduction

## Overview

ROS 2 (Robot Operating System 2) is not an operating system but rather a middleware framework that provides services designed for a heterogeneous computer cluster. It includes hardware abstraction, device drivers, libraries, visualizers, message-passing, package management, and more.

The three fundamental concepts in ROS 2 are:

- **Nodes**: Processes that perform computation. They are the basic building blocks of a ROS 2 system.
- **Topics**: Communication channels where messages are published by publishers and received by subscribers.
- **Services**: Synchronous request/response communication pattern between service clients and service servers.

These concepts enable distributed computation across multiple processes and potentially multiple devices, making ROS 2 a powerful framework for robotic applications.
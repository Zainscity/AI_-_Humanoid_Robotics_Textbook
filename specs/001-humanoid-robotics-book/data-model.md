# Data Model for "Physical AI & Humanoid Robotics Book"

This document describes the structure of the book and its components.

## Entities

### Book

The top-level entity that represents the entire book.

- **Attributes**:
  - `title`: The title of the book ("Physical AI & Humanoid Robotics — From Digital Intelligence to Embodied Agents").
  - `modules`: A collection of `Module` entities.
  - `capstoneProject`: A `CapstoneProject` entity.
  - `appendices`: A collection of `Appendix` entities.

### Module

A major section of the book that covers a broad topic.

- **Attributes**:
  - `title`: The title of the module (e.g., "ROS 2: The Robotic Nervous System").
  - `chapters`: A collection of `Chapter` entities.

### Chapter

A single chapter within a module.

- **Attributes**:
  - `title`: The title of the chapter.
  - `content`: The text of the chapter in Markdown format.
  - `codeSamples`: A collection of code snippets.
  - `diagrams`: A collection of diagrams and illustrations.

### CapstoneProject

The final project of the book.

- **Attributes**:
  - `title`: The title of the capstone project.
  - `content`: The text of the project description in Markdown format.
  - `code`: The source code for the project.

### Appendix

A supplementary section of the book.

- **Attributes**:
  - `title`: The title of the appendix (e.g., "Hardware Guide").
  - `content`: The text of the appendix in Markdown format.

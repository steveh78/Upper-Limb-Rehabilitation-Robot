# ⚙️ Upper limb rehabilitation robot

#### It is a rehabilitation robot for upper limb strength training, specifically rotational and translational movements.

<br> 

## File description

- 📂 firmware : stm32 project files, program manual(korean)
- 📂 hardware : circuit diagram, hardware manual(korean)

<br>

## Overview of research and development
### 1. Introduction
- Upper limb strength training is important for daily life and conservative treatment for patients with spinal cord injuries.
- Rotational exercises strengthen the muscles for wheelchair handling.
- Translational exercises strengthen the pushing and pulling muscles necessary for daily activities.

### 2. Design of mechanical parts
- Mechanism layout
<img width="1193" height="544" alt="mechanism" src="https://github.com/user-attachments/assets/b88d353e-c817-4903-98bc-1c76353eee27" />

### 3. Design of controller
- Block diagram
<img width="879" height="559" alt="blockdiagram" src="https://github.com/user-attachments/assets/728bd88b-9dad-42b1-8ef0-d15212de0803" />

- Circuit diagram
<img width="1327" height="583" alt="circuit" src="https://github.com/user-attachments/assets/58fc65c0-87b9-435c-b162-4ef3a69a10fe" />

### 4. Control program development
- Function : It consists of 2 exercise modes, 2 option modes, and 3 levels.

|exercise mode|option mode|level|
| --- | --- | --- |
| Rotational | Active-assist | 1, 2, 3 |
| Rotational | Resist | 1, 2, 3 |
| Translational | Active-assist | 1, 2, 3 |
| Translational | Resist | 1, 2, 3 |

- Exercise modes : Rotational and Translational exercise mode
<img width="1163" height="417" alt="exercisemodes" src="https://github.com/user-attachments/assets/d0beefe5-93ad-46ff-8dc3-27e3fc80effe" />

- Active-assist mode : It operates a motor to assist with exercise. It can also be used for muscle relaxation (stretching).
- Resist mode : It uses a motor to generate physical load for muscle strengthening.
- Levels : Exercise intensity can be adjusted in three levels : low(1), medium(2), and high(3).

#### 1. Rotational Active-assist mode
#### 2. Rotational Resist mode 
#### 3. Translational Active-assist mode 
#### 4. Translational Resist mode 

<br>

## Result
### 1. Mechanical and electrical parts
<img width="1218" height="556" alt="prototype" src="https://github.com/user-attachments/assets/16ea3694-f80c-4754-b94e-7d63b3cccfd5" />

### 2. Control program demonstration
- Rotational exercise

- Translational exercise

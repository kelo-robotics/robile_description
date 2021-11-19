# robile_description

Robot description for the KELO robile's in form of URDF files and meshes

## Available KELO ROBILE bricks

The following robile bricks are made available as [xacro macro](http://wiki.ros.org/xacro#Macros)'s. To reduce the computational load during simulation and to protect proprietary design, the meshes provided in this package are an approximation of the real hardware. The bricks are defined in the [urdf/robile_bricks](urdf/robile_bricks/) directory.

| ROBILE brick         | Purpose                                                                                                                             |
|-----------------------|-------------------------------------------------------------------------------------------------------------------------------------|
| robile_active_wheel   | A robile brick with an [active KELO Drive](https://www.shop.kelo-robotics.com/product-page/active-wheel) attached to it.           |
| robile_passive_wheel  | A robile brick with a [passive caster wheel](https://www.shop.kelo-robotics.com/product-page/passive-caster-wheel) attached to it. |
| robile_master_battery | A mockup of a robile brick with a [master battery](https://www.shop.kelo-robotics.com/product-page/onboard-cpu)                    |
| robile_cpu            | A mockup of a robile brick with an [onboard CPU](https://www.shop.kelo-robotics.com/product-page/onboard-cpu)                             |
| robile_empty          | A mockup of a full-sized [empty robile brick](https://www.shop.kelo-robotics.com/product-page/full-sized-empty-brick)              |

## Building a custom ROBILE platform configuration

The above ROBILE bricks can be used to construct customized platform's. We describe the procedure to build a new platform with a simple example. Lets assume we want to build a new platform with the name `simple_config`. This platform consists of two active wheels and two passive wheels with the following arrangement. For simplicity, lets assume we do not include any CPU or master_battery bricks in the platform. <br> ![Simple config arrangement](docs/images/simple_config_example_layout.png)

### Step-1: Define a new robot
1. Since we want to name our platform `simple_config`, we create a file `simple_config.urdf.xacro` in the [robots/](robots/) directory.
2. We then add the following block of code to define the new robot platform
    ~~~ xml
    <?xml version='1.0'?>
    <robot xmlns:xacro="http://ros.org/wiki/xacro" name="simple_config" >

        <!-- Include desired robile bricks -->
        <xacro:include filename="$(find robile_description)/urdf/robile_bricks/robile_active_wheel.urdf.xacro" />
        <xacro:include filename="$(find robile_description)/urdf/robile_bricks/robile_passive_wheel.urdf.xacro" />

        <link name="base_link"/>

    </robot>
    ~~~
    Since our platform will only consists of active and passive robile bricks, we only include the `robile_active_wheel` and `robile_passive_wheel` xacro files.

### Step-2: Add ROBILE bricks to the new robot
We shall use the center of the robot as the base_link and hence, as shown in the figure, each robile brick center will be at `+/- 0.1165m` from the base_link, depending on the position of each brick.

Therefore we instantiate the robile bricks as follows. We can use arbitrary names for each robile brick, but in this example, we name each brick as `robile_'n'`. The origin for each robile brick is specified w.r.t. the `base_link`.

~~~ xml
<!-- Build platform using robile bricks -->
<xacro:robile_passive_wheel name="robile_1" parent="base_link">
    <origin xyz="0.1165 0.1165 0.05" rpy="0.0 0.0 0.0"/>
</xacro:robile_passive_wheel>

<xacro:robile_passive_wheel name="robile_2" parent="base_link">
    <origin xyz="0.1165 -0.1165 0.05" rpy="0.0 0.0 0.0"/>
</xacro:robile_passive_wheel>

<xacro:robile_active_wheel name="robile_3" parent="base_link">
    <origin xyz="-0.1165 -0.1165 0.05" rpy="0.0 0.0 0.0"/>
</xacro:robile_active_wheel>

<xacro:robile_active_wheel name="robile_4" parent="base_link">
    <origin xyz="-0.1165 0.1165 0.05" rpy="0.0 0.0 0.0"/>
</xacro:robile_active_wheel>
~~~

### The complete platform config file

The complete platform configuration file `robots/simple_config.urdf.xacro` would look as follows:

~~~ xml
<?xml version='1.0'?>
<robot xmlns:xacro="http://ros.org/wiki/xacro" name="simple_config" >

    <!-- Include desired robile bricks -->
    <xacro:include filename="$(find robile_description)/urdf/robile_bricks/robile_active_wheel.urdf.xacro" />
    <xacro:include filename="$(find robile_description)/urdf/robile_bricks/robile_passive_wheel.urdf.xacro" />

    <link name="base_link"/>

    <!-- Build platform using robile bricks -->
    <xacro:robile_passive_wheel name="robile_1" parent="base_link">
        <origin xyz="0.1165 0.1165 0.05" rpy="0.0 0.0 0.0"/>
    </xacro:robile_passive_wheel>

    <xacro:robile_passive_wheel name="robile_2" parent="base_link">
        <origin xyz="0.1165 -0.1165 0.05" rpy="0.0 0.0 0.0"/>
    </xacro:robile_passive_wheel>

    <xacro:robile_active_wheel name="robile_3" parent="base_link">
        <origin xyz="-0.1165 -0.1165 0.05" rpy="0.0 0.0 0.0"/>
    </xacro:robile_active_wheel>

    <xacro:robile_active_wheel name="robile_4" parent="base_link">
        <origin xyz="-0.1165 0.1165 0.05" rpy="0.0 0.0 0.0"/>
    </xacro:robile_active_wheel>

</robot>
~~~

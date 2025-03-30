// SPDX-License-Identifier: GPL-2.0-only
/*
 * Driver for Linear Technology ISM6636 chip
 *
 */

 #include <linux/string.h>
 #include <linux/kernel.h>
 #include <linux/module.h>
 #include <linux/init.h>
 #include <linux/err.h>
 #include <linux/slab.h>
 #include <linux/i2c.h>
 #include <linux/hwmon.h>
 #include <linux/hwmon-sysfs.h>
 #include <linux/jiffies.h>
 #include <linux/of_device.h>
 
 #define ISM6636_PRODUCTID_REG   0x0A
 #define ISM6636_ICVERSION_REG   0x0B
 #define ISM6636_VIN_REG 	 	0x0C
 #define ISM6636_VOUT_REG	 	0x0D
 #define ISM6636_IOUT_REG	 	0x0E
 #define ISM6636_TEMP_REG     	0x0F
 #define ISM6636_RAMP_AMPLITUDE 	0x11
 
 #define DRVNAME "ism6636x"
 
 /* Here are names of the chip's registers (a.k.a. commands) */
 
 enum chips { ism6636a,ism6636c };
 
 enum ism6636_cmd {
     ism6636_vin				= 0x00, /* ro */
     ism6636_vout			= 0x01, /* ro */
     ism6636_iout			= 0x02, /* ro */
     ism6636_temp			= 0x03, /* ro */
     ism6636_pout			= 0x04,
 };
 

 struct ism6636_data {
     struct i2c_client *client;
 
     struct mutex update_lock;
     bool valid;
     unsigned long last_updated; /* in jiffies */
 
     /* Registers */
     u8 regs[5];
     u8 ramp_amplitude;
 };
 
 static struct ism6636_data *ism6636_update_device(struct device *dev)
 {
     struct ism6636_data *data = dev_get_drvdata(dev);
     struct i2c_client *client = data->client;
     s32 val;
     int i;
 
     mutex_lock(&data->update_lock);
 
     /* The chip's A/D updates 10 times per second */
     if (time_after(jiffies, data->last_updated + HZ / 10) || !data->valid) {
         //dev_dbg(&client->dev, "Starting ism6636 update\n");
         /* Read all registers */
         for (i = 0; i < (ARRAY_SIZE(data->regs)-1); i++) {
             val = i2c_smbus_read_byte_data(client, (i+ISM6636_VIN_REG) );
             if (unlikely(val < 0))
                 data->regs[i] = 0;
             else
                 data->regs[i] = val;
         }
         
         data->regs[ism6636_pout] = data->regs[ism6636_vout] * data->regs[ism6636_iout] ;
 
         data->last_updated = jiffies;
         data->valid = true;
     }
 
     mutex_unlock(&data->update_lock);
 
     return data;
 }
 
 
 
 /* Return the voltage from the given register in millivolts */
 static int ism6636_get_voltage(struct device *dev, u8 reg)
 {
     struct ism6636_data *data = ism6636_update_device(dev);
     const u8 regval = data->regs[reg];
     u32 voltage = 0;
 
     switch (reg) {
     case ism6636_vin:
         /* 62.5 mV per increment*/
         voltage = regval * 625 / 10;
         break;
     case ism6636_vout:
         /*detect ISM6636A&B or ISM6636C*/
         if( data->ramp_amplitude == 0 )
             voltage = regval * 10  + 600;
         else 
             voltage = regval * 10  + 300;
         
         break;
     default:
         /* If we get here, the developer messed up */
         WARN_ON_ONCE(1);
         break;
     }
 
     return voltage;
 }
 
 /* Return the current from the sense resistor in mA */
 static int ism6636_get_current(struct device *dev)
 {
     struct ism6636_data *data = ism6636_update_device(dev);
 
     int curr = data->regs[ism6636_iout] * 40 -2500;
 
     curr = (curr>0) ? curr:0;
 
     return curr;
 }
 
 static int ism6636_get_temp(struct device *dev)
 {
     struct ism6636_data *data = ism6636_update_device(dev);
 
     int temp = data->regs[ism6636_temp] * 1000 -75000;
 
     temp = (temp>0) ? temp:0;
 
     return temp;
 }
 
 
 static ssize_t ism6636_voltage_show(struct device *dev,
                     struct device_attribute *da, char *buf)
 {
     struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
     const int voltage = ism6636_get_voltage(dev, attr->index);
 
     return sysfs_emit(buf, "%d\n", voltage);
 }
 
 static ssize_t ism6636_current_show(struct device *dev,
                     struct device_attribute *da, char *buf)
 {
     int curr = ism6636_get_current(dev);
 
     return sysfs_emit(buf, "%d\n", curr);
 }
 
 static ssize_t ism6636_power_show(struct device *dev,
                   struct device_attribute *da, char *buf)
 {
     int curr = ism6636_get_current(dev);
     const int output_voltage = ism6636_get_voltage(dev, ism6636_vout);
 
     /* current in mA * voltage in mV == power in uW */
     int power = abs(output_voltage * curr);
 
     return sysfs_emit(buf, "%d\n", power);
 }
 
 static ssize_t ism6636_temp_show(struct device *dev,
     struct device_attribute *da, char *buf)
 {
     const int temp = ism6636_get_temp(dev);
 
     return sysfs_emit(buf, "%d\n", temp);
 }
 
 /*
  * These macros are used below in constructing device attribute objects
  * for use with sysfs_create_group() to make a sysfs device file
  * for each register.
  */
 
 /* Construct a sensor_device_attribute structure for each register */
 
 /* Current */
 static SENSOR_DEVICE_ATTR_RO(curr1_input, ism6636_current, 0);
 
 /* Input Voltage */
 static SENSOR_DEVICE_ATTR_RO(in1_input, ism6636_voltage, 0);
 /* Output Voltage */
 static SENSOR_DEVICE_ATTR_RO(in2_input, ism6636_voltage, 1);
 
 /* Temperature*/
 
 static SENSOR_DEVICE_ATTR_RO(temp1_input, ism6636_temp, 0);
 
 /* Power  Input*/
 static SENSOR_DEVICE_ATTR_RO(power1_input, ism6636_power, 0);
 
 /*
  * Finally, construct an array of pointers to members of the above objects,
  * as required for sysfs_create_group()
  */
 static struct attribute *ism6636_attrs[] = {
 
     &sensor_dev_attr_curr1_input.dev_attr.attr,
     &sensor_dev_attr_in1_input.dev_attr.attr,
     &sensor_dev_attr_in2_input.dev_attr.attr,
 
     &sensor_dev_attr_temp1_input.dev_attr.attr,
     &sensor_dev_attr_power1_input.dev_attr.attr,
     NULL,
 };
 ATTRIBUTE_GROUPS(ism6636);
 
 static int ism6636_probe(struct i2c_client *client)
 {
     struct i2c_adapter *adapter = client->adapter;
     struct device *dev = &client->dev;
     struct ism6636_data *data;
     struct device *hwmon_dev;
 
     if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA))
         return -ENODEV;
 
     data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
     if (!data)
         return -ENOMEM;
 
     data->client = client;
 
 	  const struct of_device_id *match;
     
     match = i2c_of_match_device(client->dev.driver->of_match_table, client);
     if(!match)
     {
         dev_err(&client->dev, "ism6636 no matching of_device_id\n");
         return -ENODEV;
     }
 
     const char *compatible_str = match->compatible;
     printk("ism 6636 :match driver name = %s\n",compatible_str);
 
     
     
     if(strcmp(compatible_str,"ism,ism6636a") ==0 )
     {
         data->ramp_amplitude =(u8)ism6636a; 
     }
	  else if (strcmp(compatible_str,"ism,ism6636c") == 0 )
	  {
	      data->ramp_amplitude =(u8)ism6636c;
	  }
     printk("ism 6636 :match data->ramp_amplitude = %u\n",data->ramp_amplitude);
     
     
     mutex_init(&data->update_lock);
 
     /* Initialize the ism6636 chip */
     //i2c_smbus_write_byte_data(client, LTC4215_FAULT, 0x00);
 
     //get ramp_amplitude
     //data->ramp_amplitude = i2c_smbus_read_byte_data(client, ISM6636_RAMP_AMPLITUDE);
 
     hwmon_dev = devm_hwmon_device_register_with_groups(dev, client->name,
                                data,
                                ism6636_groups);
 
 
 
     return PTR_ERR_OR_ZERO(hwmon_dev);
 }
 
 static const struct i2c_device_id ism6636_id[] = {
     { "ism6636a", ism6636a },
     { "ism6636c", ism6636c }
 };

 //MODULE_DEVICE_TABLE(i2c, ism6636_id);
 
 static const struct of_device_id ism6636_of_match[] = {
     { .compatible = "ism,ism6636a",.data = (void *)ism6636a, },
     { .compatible = "ism,ism6636c",.data = (void *)ism6636c, },
     {}
 };
 
 MODULE_DEVICE_TABLE(of, ism6636_of_match);
 
 /* This is the driver that will be inserted */
 static struct i2c_driver ism6636_driver = {
     .driver = {
         .owner = THIS_MODULE,
         .name = DRVNAME,
         .of_match_table = of_match_ptr(ism6636_of_match),
     },
     .class		= I2C_CLASS_HWMON,
     .probe		= ism6636_probe,
     .id_table	= ism6636_id,
 };
 
 module_i2c_driver(ism6636_driver);
 
 MODULE_AUTHOR("Shaojie Zhang");

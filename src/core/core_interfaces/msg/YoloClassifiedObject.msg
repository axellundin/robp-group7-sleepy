geometry_msgs/PoseStamped center_point #position of object center point with timestamp
geometry_msgs/PoseStamped topleft_point #position of object topleft corner point with timestamp
geometry_msgs/PoseStamped bottomright_point #position of object bottomright_point center point with timestamp
string category  #name of category
int32 category_num    
sensor_msgs/Image image #cropped image of the object inside the box  
geometry_msgs/Point[] points  
geometry_msgs/Point[] colors  
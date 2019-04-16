#include <iostream>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>

//#include <pcl/io/point_cloud_image_extractors.h>
using namespace std;
using namespace cv;



// salva em formato pdc pontos sem RGB
// fonte https://github.com/IntelRealSense/librealsense/blob/master/wrappers/pcl/pcl/rs-pcl.cpp
void salve_points(const rs2::points& points);
// salva em formato png para color e csv para ponto
void savle_metadata_to_csv_png(const rs2::frame& frm, const std::string& filename);
// conver pcd com cor
void salve_cloud_color(const rs2::points& points, const rs2::video_frame& color);
// usada para obter cor emconverte_tram_to_cloud_png
std::tuple<int, int, int> RGB_Texture(rs2::video_frame texture, rs2::texture_coordinate Texture_XY);
void converte_point_cloud_to_png( pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
int main()try
{

    // Declare pointcloud object, for calculating pointclouds and texture mappings
    rs2::pointcloud pc;
    // We want the points object to be persistent so we can display the last cloud when a frame drops
    rs2::points points;

    rs2::colorizer color_map;
    rs2::config cfg;
    cfg.enable_device_from_file("/home/elton/Downloads/d435i_sample_data/d435i_walk_around.bag");
    auto pipe=std::make_shared<rs2::pipeline>();
    pipe->start(cfg);// abre para leitura
    auto device=pipe->get_active_profile().get_device();
    rs2::frame color;
    rs2::frameset data;
    auto depth = data.as<rs2::video_frame>();

    if(device.as<rs2::playback>()){
        printf("device como reprodutor \n");
        rs2::playback playback = device.as<rs2::playback>();

        cout <<"posicao" <<playback.get_position() <<endl;

        playback.set_real_time(true);//nao pula frame

        playback.resume();// inicia reproducao
        //  playback.seek(std::chrono::seconds(1));//pula para x segundo

        cout <<"estatus = " << playback.current_status() <<endl;
        //      playback.set_playback_speed(0.1f);// velocidade de reproducao
        int64_t temp_frame=0;
        while (waitKey(1) !=27 ){

            if(pipe->poll_for_frames(&data)){
                // cout <<"posicao" <<playback.get_position() <<endl;

                depth = data.get_depth_frame();//.apply_filter(color_map);
                color = data.get_color_frame();

                // para salvar pcd colorido
                    pc.map_to(color);
                   points = pc.calculate(depth);

              //  depth = depth.apply_filter(color_map);

                playback.pause();

                // cout <<"hora color" <<color.get_timestamp()<<endl; // hora de captura milsegundo
                //  cout <<"hora depth" <<depth.get_timestamp()<<endl; // hora de captura
                // int64 tempo =depth.get_timestamp();
                //  playback.seek(std::chrono::milliseconds(tempo));
                const int w =depth.as<rs2::video_frame>().get_width();
                const int h =depth.as<rs2::video_frame>().get_height();
                const int wc =color.as<rs2::video_frame>().get_width();
                const int hc =color.as<rs2::video_frame>().get_height();

                Mat image(Size(w,h), CV_8UC3,(void*)depth.get_data(),Mat::AUTO_STEP);
                Mat image_clor(Size(wc,hc), CV_8UC3,(void*)color.get_data(),Mat::AUTO_STEP);

                imshow("color",image_clor);
                imshow("depth",image);
                //                cout << image<<endl;

                auto clik = waitKey(0);
                cout <<clik<<endl;
                switch (clik) {
                case 81: cout<<"voltar "<<temp_frame<<endl;

                    playback.seek(std::chrono::nanoseconds(temp_frame));
                    break;
                case 83: cout << "proximo"<<endl;
                    auto t= std::chrono::nanoseconds(1);
                    temp_frame= depth.get_timestamp();
                 //case 27:

                }

                cout <<temp_frame<<endl;
                playback.resume();
            }else cout <<"sem fram"<<endl;
        }
    }
   // depth= color_map.process(data);// para csv
    // Record per-frame metadata for UVC streams
                std::stringstream csv_file;
                csv_file << "rs-save-to-disk-output-" << depth.get_profile().stream_name()
                         << "-metadata.csv";
    savle_metadata_to_csv_png(depth,csv_file.str());

     salve_cloud_color(points,color);
    pipe->stop();//fecha o arquivo

    cout << "fim" << endl;
    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
void savle_metadata_to_csv_png(const rs2::frame& frm, const std::string& filename)
{

    std::ofstream csv;

    csv.open(filename);

    //    std::cout << "Writing metadata to " << filename << endl;
    csv << "Stream," << rs2_stream_to_string(frm.get_profile().stream_type()) << "\nMetadata Attribute,Value\n";

    // Record all the available metadata attributes
    for (size_t i = 0; i < RS2_FRAME_METADATA_COUNT; i++)
    {
        if (frm.supports_frame_metadata((rs2_frame_metadata_value)i))
        {
            csv << rs2_frame_metadata_to_string((rs2_frame_metadata_value)i) << ","
                << frm.get_frame_metadata((rs2_frame_metadata_value)i) << "\n";
        }
    }

    csv.close();
}

void salve_points(const rs2::points& points)
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    cloud->width = sp.width();
    cloud->height = sp.height();
    cloud->is_dense = false;
    cloud->points.resize(points.size());
    auto ptr = points.get_vertices();
    for (auto& p : cloud->points)
    {
        p.x = ptr->x;
        p.y = ptr->y;
        p.z = ptr->z;
        ptr++;
    }
    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZ> ("arquivo.pcd", *cloud, false);


}
void converte_point_cloud_to_png( pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){

    cv::Mat image = cv::Mat(cloud->height, cloud->width, CV_8UC3);
    for (int i = 0; i < image.rows; i++)
    {
        for (int j = 0; j < image.cols; j++)
        {
            pcl::PointXYZRGB point = cloud->at(j, i);
            image.at<cv::Vec3b>(i, j)[0] = point.b;
            image.at<cv::Vec3b>(i, j)[1] = point.g;
            image.at<cv::Vec3b>(i, j)[2] = point.r;
        }
    }
    cv::imwrite("image.png", image);
   // imshow("image do",image);

}
void salve_cloud_color(const rs2::points& points, const rs2::video_frame& color){

    // Object Declaration (Point Cloud)
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    // Declare Tuple for RGB value Storage (<t0>, <t1>, <t2>)
    std::tuple<uint8_t, uint8_t, uint8_t> RGB_Color;

    //================================
    // PCL Cloud Object Configuration
    //================================
    // Convert data captured from Realsense camera to Point Cloud
    auto sp = points.get_profile().as<rs2::video_stream_profile>();

    cloud->width  = static_cast<uint32_t>( sp.width()  );
    cloud->height = static_cast<uint32_t>( sp.height() );
    cloud->is_dense = false;
    cloud->points.resize( points.size() );

    auto Texture_Coord = points.get_texture_coordinates();
    auto Vertex = points.get_vertices();

    // Iterating through all points and setting XYZ coordinates
    // and RGB values
    for (int i = 0; i < points.size(); i++)
    {
        //===================================
        // Mapping Depth Coordinates
        // - Depth data stored as XYZ values
        //===================================
        cloud->points[i].x = Vertex[i].x;
        cloud->points[i].y = Vertex[i].y;
        cloud->points[i].z = Vertex[i].z;

        // Obtain color texture for specific point
        RGB_Color = RGB_Texture(color, Texture_Coord[i]);

        // Mapping Color (BGR due to Camera Model)
        cloud->points[i].r = get<2>(RGB_Color); // Reference tuple<2>
        cloud->points[i].g = get<1>(RGB_Color); // Reference tuple<1>
        cloud->points[i].b = get<0>(RGB_Color); // Reference tuple<0>

    }
    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZRGB> ("arquivo.pcd", *cloud, false);
    converte_point_cloud_to_png(cloud);//gera png apartir de claud
}

//======================================================
// RGB Texture
// - Function is utilized to extract the RGB data from
// a single point return R, G, and B values.
// Normals are stored as RGB components and
// correspond to the specific depth (XYZ) coordinate.
// By taking these normals and converting them to
// texture coordinates, the RGB components can be
// "mapped" to each individual point (XYZ).
//======================================================
std::tuple<int, int, int> RGB_Texture(rs2::video_frame texture, rs2::texture_coordinate Texture_XY)
{
    // Get Width and Height coordinates of texture
    int width  = texture.get_width();  // Frame width in pixels
    int height = texture.get_height(); // Frame height in pixels

    // Normals to Texture Coordinates conversion
    int x_value = min(max(int(Texture_XY.u * width  + .5f), 0), width - 1);
    int y_value = min(max(int(Texture_XY.v * height + .5f), 0), height - 1);

    int bytes = x_value * texture.get_bytes_per_pixel();   // Get # of bytes per pixel
    int strides = y_value * texture.get_stride_in_bytes(); // Get line width in bytes
    int Text_Index =  (bytes + strides);

    const auto New_Texture = reinterpret_cast<const uint8_t*>(texture.get_data());

    // RGB components to save in tuple
    int NT1 = New_Texture[Text_Index];
    int NT2 = New_Texture[Text_Index + 1];
    int NT3 = New_Texture[Text_Index + 2];

    return std::tuple<int, int, int>(NT1, NT2, NT3);
}


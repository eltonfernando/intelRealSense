// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
//#include <librealsense2/rsutil.h>
#include <opencv2/opencv.hpp>

//#include "/home/embrapa/librealsense/examples/example.hpp"          // Include short list of convenience functions for rendering

// This example will require several standard data-structures and algorithms:
//#define _USE_MATH_DEFINES
//#include <math.h>
//#include <queue>
//#include <unordered_set>
//#include <map>
//#include <thread>
//#include <atomic>
//#include <mutex>
using namespace cv;

int main(int argc, char * argv[]) try
{
    // OpenGL textures for the color and depth frames
   // texture depth_image, color_image;

    const auto window_name = "Display image";
    namedWindow(window_name,WINDOW_AUTOSIZE);
    // Colorizer is used to visualize depth data
    rs2::colorizer color_map;
    // Use black to white color map
    color_map.set_option(RS2_OPTION_COLOR_SCHEME, 2.f);
    // Filtto decimação Decimation filter reduces the amount of data (while preserving best samples)
    rs2::decimation_filter filtro_dec;
    // If the demo is too slow, make sure you run in Release (-DCMAKE_BUILD_TYPE=Release)
    // but you can also increase the following parameter to decimate depth more (reducing quality)
    // 2 é fator de escala (exemplo para 1280X720 e fator 2 a imagem tera 640x360)
    filtro_dec.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2);
    // Define transformations from and to Disparity domain
    rs2::disparity_transform depth2disparity;
    rs2::disparity_transform disparity2depth(false);
    // Define spatial filter (edge-preserving)
    rs2::spatial_filter filtro_spat;
    // Enable hole-filling
    // Hole filling is an agressive heuristic and it gets the depth wrong many times
    // (the shortest-path will always prefer to "cut" through the holes since they have zero 3D distance)
    filtro_spat.set_option(RS2_OPTION_FILTER_MAGNITUDE,2);//numero de interacao //range [1-5] default 2
    filtro_spat.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA,0.5); //fator de utilizacao 1=sem filtro 0= filtro infinito range[0.25-1] default =0.55
    filtro_spat.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA,20); //tamanho do passo: estabelece limite usado para preservar borda range [1-50] dafault 20
    filtro_spat.set_option(RS2_OPTION_HOLES_FILL, 3); // preenchemento de furo 0= nem um 5= todos pixels range [0-5] intervalo nao mapeado [nem um, 2,4,5,16,infinito]pixels

    // Define temporal filter
    rs2::temporal_filter filtro_temp;
    filtro_temp.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA,0.4f);// fator de utilizacao range[0-1] default 0.4
    filtro_temp.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA,20); //
 // alinhar espacialmento todos fluxo para a viewport de profudidade
    rs2::align align_to(RS2_STREAM_DEPTH);

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;

    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_DEPTH); // Enable default depth
    // Para o fluxo de cores, defina o formato para RGBA
    // resolusão
    // formatos para opencv use RS2_FROMAT_BGR8 ver em https://intelrealsense.github.io/librealsense/doxygen/rs__sensor_8h.html#a01b4027af33139de861408872dd11b93
    // taxa de amostragem
   // cfg.enable_stream(RS2_STREAM_COLOR, 1920, 1080, RS2_FORMAT_RGBA8, 15);
     cfg.enable_stream(RS2_STREAM_COLOR, 848,480, RS2_FORMAT_BGR8, 15);
    auto profile = pipe.start(cfg);

    // configurando para alta precisão D400 stereoscopic cameras
    // TODO: SR300 structured-light cameras and the L500 would require different handling
    auto sensor = profile.get_device().first<rs2::depth_stereo_sensor>();
    if (sensor)
    {
        sensor.set_option(RS2_OPTION_VISUAL_PRESET, RS2_RS400_VISUAL_PRESET_HIGH_ACCURACY);
    }

 // auto stream = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();

while(waitKey(1)<0 && getWindowProperty(window_name,WND_PROP_AUTOSIZE)>=0)
{

   rs2::frameset data =  pipe.wait_for_frames();

        // First make the frames spatially aligned
        data = data.apply_filter(align_to);

        // Decimation will reduce the resultion of the depth image,
        // closing small holes and speeding-up the algorithm
        data = data.apply_filter(filtro_dec);

        // To make sure far-away objects are filtered proportionally
        // we try to switch to disparity domain
        data = data.apply_filter(depth2disparity);

        // Apply spatial filtering
        data = data.apply_filter(filtro_spat);

        // Apply temporal filtering
       data = data.apply_filter(filtro_temp);

        // If we are in disparity domain, switch back to depth
        data = data.apply_filter(disparity2depth);

        //// Apply color map for visualization of depth
      //  data = data.apply_filter(color_map);


  //  if (current_frameset)
  //  {
        auto depth = data.get_depth_frame();
        auto color = data.get_color_frame();
       // auto colorized_depth = data.first(RS2_STREAM_DEPTH, RS2_FORMAT_RGB8);
        auto colorized_depth = data.first(RS2_STREAM_DEPTH, RS2_FORMAT_BGR8);
  //  }
        const int w =data.as<rs2::video_frame>().get_width();
        const int h =data.as<rs2::video_frame>().get_height();
             // criando matrix com openvn
               Mat image(Size(w,h), CV_8UC3,(void*)depth.get_data(),Mat::AUTO_STEP);

               //Atualiza frame na tela
               imshow(window_name,image);

}

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



#include <iostream>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;
int main()
{
    rs2::colorizer color_map;

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
    rs2::config cfg;
    cfg.enable_record_to_file("output_file.bag");
    cfg.enable_stream(RS2_STREAM_DEPTH); // Enable default depth
    // For the color stream, set format to RGBA
    // To allow blending of the color frame on top of the depth frame
    cfg.enable_stream(RS2_STREAM_COLOR);//, 1920, 1080, RS2_FORMAT_RGBA8, 15);
    rs2::pipeline pipe;
    pipe.start(cfg);// a captura inicia aqui

    while (waitKey(1)!=27) {
        auto data= pipe.wait_for_frames();

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

        auto depth = data.get_depth_frame().apply_filter(color_map);
        auto color= data.get_color_frame();

        const int w = depth.as<rs2::video_frame>().get_width();
        const int h = depth.as<rs2::video_frame>().get_height();
        const int wc =color.as<rs2::video_frame>().get_width();
        const int hc =color.as<rs2::video_frame>().get_height();

        // Create OpenCV matrix of size (w,h) from the colorized depth data
        Mat image(Size(w, h), CV_8UC3, (void*)depth.get_data(), Mat::AUTO_STEP);
        Mat im_color(Size(wc, hc), CV_8UC3, (void*)color.get_data(), Mat::AUTO_STEP);

      //  depth.get_di
//int d = 0;
        // Update the window with new data
//String image_depth = "depth distancia = "+std::to_string(d);
        imshow("image_depth", image);
        imshow("color",im_color);

    }
    cout << "EU NÃO TRAVEI SÓ ESTOU SALVANDO"<<endl;
    pipe.stop();//para a gravação

    cout << "ok" << endl;
    return 0;
}

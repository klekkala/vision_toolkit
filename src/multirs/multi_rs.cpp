#include <librealsense2/rs.hpp>
#include "/usr/local/include/librealsense2/rs_advanced_mode.hpp"
#include <iostream>
#include <fstream>
#include <map>
#include <thread>
#include <string>
#include <vector>

// 3rd party header for writing png files
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "/usr/include/stb/stb_image_write.h"

using namespace rs2;
using namespace std;

//---------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Capture & save function
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------

void capture(rs2::pipeline p, string serial)
{
	cout << "\nStarting capture with the camera : " << serial << endl;
	frameset fs = p.wait_for_frames();
	for (auto j = 0; j < 30; ++j) fs = p.wait_for_frames(); // Wait a bit for auto exposure to settle
	rs2::pointcloud pc;
	auto color = fs.get_color_frame();
	pc.map_to(color);
	std::stringstream png_file;
	png_file << "Image_" << serial << ".png";
	stbi_write_png(png_file.str().c_str(), color.get_width(), color.get_height(), color.get_bytes_per_pixel(), color.get_data(), color.get_stride_in_bytes());
	cout << "Saved " << png_file.str() << endl;
}

//---------------------------------------------------------------------------------------------------------------------------------------------------------------------
// This function will go through every connected device (as long as it is a D455), apply a custom ".json", and then capture & save a ".png" for each device simultaneously.
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------

int main() try
{
	context ctx;
	vector<thread> threads;
	map<string, pipeline> pipes;
	cout << "new" << endl;
	for (auto dev : ctx.query_devices()) // For each device do :
	{
		cout << "Blah.." << dev.get_info(RS2_CAMERA_INFO_NAME) << endl;
		if (strcmp(dev.get_info(RS2_CAMERA_INFO_NAME), "Intel RealSense D455") == 0) // Check for compatibility
		{
			pipeline p;
			config cfg;
			string serial = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
			string json_file_name = "9.json";
			cout << "Configuring camera : " << serial << endl;

			// Add desired stream to configuration
			cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_RGB8, 5);
			
			cout << "Stream enabled" << endl;
			auto advanced_mode_dev = dev.as<rs400::advanced_mode>();
			
			// Check if advanced-mode is enabled to pass the custom config
			if (!advanced_mode_dev.is_enabled())
			{
				// If not, enable advanced-mode
				advanced_mode_dev.toggle_advanced_mode(true);
				cout << "Advanced mode enabled. " << endl;
			}

			// Select the custom configuration file
			std::ifstream t(json_file_name);
			std::string preset_json((std::istreambuf_iterator<char>(t)), std::istreambuf_iterator<char>());
			advanced_mode_dev.load_json(preset_json);
			cfg.enable_device(serial);

			cout << "advanced mode done" << endl;
			// Start the pipeline
			p.start(cfg);
			cout << "Starting pipeline for current camera " << serial << endl;
			pipes[serial] = p;
		}
	}

	cout << "Starting pushback" << endl;
	// Call the Capture & save function to start simultaneous threads
	for (map<string, pipeline>::iterator it = pipes.begin(); it != pipes.end(); ++it)
	{
		threads.push_back(std::thread(capture, it->second, it->first));
	}

	// Joining the threads
	for (auto& t : threads) 
	{
		t.join();
	}

	return EXIT_SUCCESS;
}

catch (const rs2::error & e)
{
	std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
	system("pause");
	return EXIT_FAILURE;
}

catch (const std::exception& e)
{
	std::cerr << e.what() << std::endl;
	system("pause");
	return EXIT_FAILURE;
}


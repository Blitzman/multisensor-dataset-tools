#include "Utils.h"

void Utils::get_pcd_filenames
	(
		const std::string & pDirectoryPath, 
		std::vector<std::string> & pFilenames
	)
{
	boost::filesystem::path directory(pDirectoryPath);
	pFilenames.clear();

	try
	{
		if (boost::filesystem::exists(directory))
		{
			if (boost::filesystem::is_directory(directory))
			{
				std::vector<boost::filesystem::path> paths;
				std::copy
					(
						boost::filesystem::directory_iterator(directory), 
						boost::filesystem::directory_iterator(), 
						std::back_inserter(paths)
					);
				std::sort(paths.begin(), paths.end());

				for (std::vector<boost::filesystem::path>::const_iterator it = paths.begin(); it != paths.end(); ++it)
				{
					#ifdef DEBUG
						std::cout << *it << "\n";
					#endif

            pFilenames.push_back((*it).string());
				}
			}
		}
	}
	catch (const boost::filesystem::filesystem_error& ex)
	{
		std::cerr << ex.what() << '\n';
	}
}
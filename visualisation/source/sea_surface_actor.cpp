#include "sea_surface_actor.h"
#include "exception.h"
#include <iostream>
#include <iomanip>

using namespace asv_swarm;
using namespace Visualisation;

Sea_surface_actor::Sea_surface_actor(struct Wave* wave):
  vtkPolyDataAlgorithm{},
  wave{wave},
  timer_count{0u},
  timer_step_size{0u},
  field_length {100.0}
{
  // TODO: 
  // Initialise count_rows_sea_surface_points. Should be greater than 1.
  // Initialise sea_surface_points

  // This filter does not need an input port
  SetNumberOfInputPorts(0);

  // Initialize the mapper and actor
  sea_surface_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  sea_surface_mapper->SetInputConnection(this->GetOutputPort());
  sea_surface_actor = vtkSmartPointer<vtkActor>::New();
  sea_surface_actor->SetMapper(sea_surface_mapper);
  sea_surface_actor->GetProperty()->SetRepresentationToWireframe();
  sea_surface_actor->GetProperty()->SetColor(0,0,255); // blue waves
}

int Sea_surface_actor::RequestData(vtkInformation* request,
                                   vtkInformationVector** inputVector,
                                   vtkInformationVector* outputVector)
{
  
  // Get output
  vtkPolyData* output = vtkPolyData::GetData(outputVector,0);

  // Get the timer count
  // TODO: Correct the time. It should get the repeat timer interval and
  // multiply when calculating time. Maybe a good way would be the
  // callback::Execute() set the time in sea_surface_viz and then call
  // requestdata.
  auto time = timer_count * timer_step_size; // milli-seconds

  // Set the sea surface profile for the current time step
  set_sea_surface_elevations(time);

  // Create the points, cells and mesh
  if(sea_surface_mesh_points)
  { 
    // Points already made no need to create again. Only need to modify the 
    // z coordinates.
    unsigned int sea_surface_mesh_point_id = 0u;
    for(auto& points_row : sea_surface_points)
    {
      for(auto& point : points_row)
      {
        double x = point.x;
        double y = point.y;
        //TODO: Correct the formula for z by removing the scaling factor.
        double z = point.z;
        sea_surface_mesh_points->SetPoint(sea_surface_mesh_point_id,x,y,z); 
        ++sea_surface_mesh_point_id;
      }
    }
    sea_surface_mesh_points->Modified();
    // Update the cells
    sea_surface_mesh_cells->Modified();
  }
  else
  {
    // Initialize the sea surface mesh.

    // The first step is to create the mesh to represent the sea surface. 
    // Creating the mesh contains two steps:
    // 1. Create all the control points on the mesh
    // 2. Connect the control points and create each cell of the mesh
    // Then use the points and the connection details to create the full mesh

    // Create the control points for the mesh 
    sea_surface_mesh_points = vtkSmartPointer<vtkPoints>::New();
    sea_surface_mesh_points->SetNumberOfPoints( sea_surface_grid_size * 
                                                sea_surface_grid_size);
    // set the control points for vtk mesh 
    unsigned int sea_surface_mesh_point_id = 0u;
    for(auto& points_row : sea_surface_points)
    {
      for(auto& point : points_row)
      {
        double x = point.x;
        double y = point.y;
        //TODO: Correct the formula for z by removing the scaling factor.
        double z = point.z;
        sea_surface_mesh_points->InsertPoint(sea_surface_mesh_point_id,x,y,z); 
        ++sea_surface_mesh_point_id;
      }
    }

    // Create the cells of the mesh. The cell array can be thought of as a 
    // connectivity list. Here we specify the number of points followed by that 
    // number of point ids. This can be repeated as many times as there are 
    // primitives in the list. 
    sea_surface_mesh_cells = vtkSmartPointer<vtkCellArray>::New(); // This can 
    // be considered as an array of arrays where each array represents a square 
    // mesh that is part of the large mesh.

    // Now that the container for holding each mesh primitive has been created,
    // enter each primitive into the container.
    for(unsigned int i{0u}; i<sea_surface_grid_size-1; ++i)
    {
      for(unsigned int j{0u}; j<sea_surface_grid_size-1; ++j)
      {
        unsigned int count_nodes_in_cell {3};
        sea_surface_mesh_cells->InsertNextCell(count_nodes_in_cell);
        sea_surface_mesh_cells->InsertCellPoint(i*sea_surface_grid_size+j);
        sea_surface_mesh_cells->InsertCellPoint(i*sea_surface_grid_size+j+1);
        sea_surface_mesh_cells->InsertCellPoint((i+1)*sea_surface_grid_size+j+1);

        sea_surface_mesh_cells->InsertNextCell(count_nodes_in_cell);
        sea_surface_mesh_cells->InsertCellPoint((i+1)*sea_surface_grid_size+j+1);
        sea_surface_mesh_cells->InsertCellPoint((i+1)*sea_surface_grid_size+j);
        sea_surface_mesh_cells->InsertCellPoint(i*sea_surface_grid_size+j);
      }
    }
  }
  
  // Create the mesh
  output->SetPoints(sea_surface_mesh_points);
  output->SetPolys(sea_surface_mesh_cells);
  output->Modified();
  return 1;
}

void Sea_surface_actor::set_sea_surface_elevations(unsigned long time)
{
  // TODO
}

void Sea_surface_actor::set_sea_surface_points()
{
  // Clear the control points if not empty
  if(!sea_surface_points.empty())
  {
    sea_surface_points.clear();
  }
  double patch_length = field_length / (sea_surface_grid_size - 1);
  // Create a 2D array of control points.
  for(unsigned int i = 0; i < sea_surface_grid_size; ++i)
  {
    std::vector<Dimensions> points_row;
    for(unsigned int j = 0; j < sea_surface_grid_size; ++j)
    {
      double x = patch_length * j; // m
      double y = patch_length * i; // m
      double z = 0.0; // m
      Dimensions point{x,y,z};
      points_row.push_back(point);
    }
    sea_surface_points.push_back(points_row);
  }
}

void Sea_surface_actor::set_field_length(double field_length)
{
  if( field_length <= 0.0)
  {
    throw asv_swarm::Exception::ValueError("Field length should be a positive value.");
  }
  this->field_length = field_length;
  set_sea_surface_points();
}

void Sea_surface_actor::set_sea_surface_grid_size(unsigned int grid_size)
{
  if(grid_size <= 1)
  {
    throw asv_swarm::Exception::ValueError("Sea surface grid size should be > 1");
  }
  sea_surface_grid_size = grid_size;
  set_sea_surface_points();
}
import 'package:flutter/material.dart';
import 'package:flutter_bloc/flutter_bloc.dart';
import 'package:stpvelox/core/utils/colors.dart';
import 'package:stpvelox/domain/entities/program.dart';
import 'package:stpvelox/presentation/blocs/program_selection/program_selection_bloc.dart';
import 'package:stpvelox/presentation/blocs/program_selection/program_selection_event.dart';
import 'package:stpvelox/presentation/blocs/program_selection/program_selection_state.dart';
import 'package:stpvelox/presentation/screens/program_screen.dart';
import 'package:stpvelox/presentation/widgets/grid_tile.dart';
import 'package:stpvelox/presentation/widgets/responsive_grid.dart';
import 'package:stpvelox/presentation/widgets/top_bar.dart';

class ProgramSelectionScreen extends StatelessWidget {
  ProgramSelectionScreen({super.key});

  @override
  Widget build(BuildContext context) {
    return BlocListener<ProgramSelectionBloc, ProgramSelectionState>(
      listener: (context, state) {
        if (state is ProgramTappedState) {
                    Navigator.push(
            context,
            MaterialPageRoute(builder: (_) => ProgramScreen(program: state.program)),
          );
        }
      },
      child: Scaffold(
        appBar: createTopBar(context, "Programs"),
        body: SafeArea(
          child: Column(
            children: [
              Expanded(
                child: Padding(
                  padding: const EdgeInsets.all(8.0),
                  child: BlocBuilder<ProgramSelectionBloc, ProgramSelectionState>(
                    builder: (context, state) {
                      if (state is ProgramSelectionLoading) {
                        return const Center(child: CircularProgressIndicator());
                      } else if (state is ProgramSelectionLoaded) {
                        final programs = state.programs;
                        return ResponsiveGrid(
                          children: programs.map((program) {
                            return _buildProgramTile(
                              context,
                              programs.indexOf(program),
                              program,
                            );
                          }).toList(),
                        );
                      } else if (state is ProgramSelectionError) {
                        return Center(
                          child: Text(
                            state.message,
                            style:
                            const TextStyle(color: Colors.red, fontSize: 18),
                          ),
                        );
                      } else {
                        context
                            .read<ProgramSelectionBloc>()
                            .add(LoadProgramSelectionEvent());
                        return Container();
                      }
                    },
                  ),
                ),
              ),
            ],
          ),
        ),
      ),
    );
  }

  Widget _buildProgramTile(BuildContext context, int index, Program program) {
    return ResponsiveGridTile(
      label: program.name,
      icon: Icons.code,
      onPressed: () {
        context.read<ProgramSelectionBloc>().add(ProgramTappedEvent(program: program));
      },
      color: AppColors.getTileColor(index),
    );
  }
}
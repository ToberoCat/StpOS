import 'package:flutter/material.dart';
import 'package:flutter_bloc/flutter_bloc.dart';
import 'package:stpvelox/domain/entities/args/arg.dart';
import 'package:stpvelox/domain/entities/program.dart';
import 'package:stpvelox/presentation/blocs/program/program_bloc.dart';
import 'package:stpvelox/presentation/widgets/grid_tile.dart';
import 'package:stpvelox/presentation/widgets/top_bar.dart';
import 'package:xterm/xterm.dart';

class ProgramScreen extends StatefulWidget {
  final Program program;

  const ProgramScreen({super.key, required this.program});

  @override
  State<ProgramScreen> createState() => _ProgramScreenState();
}

class _ProgramScreenState extends State<ProgramScreen> {
  OverlayEntry? overlayEntry;

  @override
  void initState() {
    super.initState();
    WidgetsBinding.instance.addPostFrameCallback((_) {
      createControlOverlay(context.read<ProgramBloc>().state);
    });
  }

  @override
  void dispose() {
    removeOverlay();
    context.read<ProgramBloc>().close();
    super.dispose();
  }

  void _onLongPress(ProgramState state) {
    createControlOverlay(state);
  }

  Widget startButton() {
    return SizedBox(
      width: 200,
      height: 200,
      child: ResponsiveGridTile(
        label: 'Start',
        icon: Icons.play_arrow,
        color: Colors.green,
        onPressed: () {
          createArgOverlay({}, widget.program, 0, widget.program.args.firstOrNull);
        },
      ),
    );
  }

  Widget stopButton() {
    return SizedBox(
      width: 200,
      height: 200,
      child: ResponsiveGridTile(
        label: 'Stop',
        icon: Icons.stop,
        color: Colors.red,
        onPressed: () {
          removeOverlay();

          context.read<ProgramBloc>().add(StopProgramEvent());
        },
      ),
    );
  }

  void createArgOverlay(
      Map<String, String> args, Program program, int idx, Arg? arg) {
    removeOverlay();
    assert(overlayEntry == null);

    if (arg == null) {
      context
          .read<ProgramBloc>()
          .add(StartProgramEvent(program: program, args: args));
      return;
    }

    overlayEntry = OverlayEntry(
      builder: (BuildContext context) {
        return GestureDetector(
          child: Material(
            color: Colors.black54,
            child: Center(
              child: Column(
                mainAxisAlignment: MainAxisAlignment.spaceEvenly,
                crossAxisAlignment: CrossAxisAlignment.center,
                children: [
                  Center(
                    child: Column(
                      children: [
                        Text(
                          arg.name,
                          style: const TextStyle(
                            fontSize: 30,
                            color: Colors.white,
                          ),
                        ),
                        const SizedBox(height: 8),
                        arg.build(context, (value) {
                          args[arg.name] = value;
                        }),
                      ],
                    ),
                  ),
                  SizedBox(
                    width: 200,
                    height: 200,
                    child: ResponsiveGridTile(
                      label: 'Submit',
                      icon: Icons.hide_image_rounded,
                      color: Colors.blue,
                      onPressed: () {
                        if (idx + 1 >= program.args.length) {
                          createArgOverlay(args, program, idx + 1, null);
                          return;
                        }

                        createArgOverlay(
                            args, program, idx + 1, program.args[idx + 1]);
                      },
                    ),
                  ),
                ],
              ),
            ),
          ),
        );
      },
    );

    Overlay.of(context).insert(overlayEntry!);
  }

  void createControlOverlay(ProgramState state) {
    removeOverlay();
    assert(overlayEntry == null);

    var running = state is ProgramStarted && state.session.isRunning;
    overlayEntry = OverlayEntry(
      builder: (BuildContext context) {
        return GestureDetector(
          child: Material(
            color: Colors.black54,
            child: Center(
              child: Row(
                mainAxisAlignment: MainAxisAlignment.spaceEvenly,
                crossAxisAlignment: CrossAxisAlignment.center,
                children: [
                  if (!running) startButton() else stopButton(),
                  SizedBox(
                    width: 200,
                    height: 200,
                    child: ResponsiveGridTile(
                      label: 'Hide',
                      icon: Icons.hide_image_rounded,
                      color: Colors.blue,
                      onPressed: () => removeOverlay(),
                    ),
                  ),
                                                                                                                                                                                                                      ],
              ),
            ),
          ),
        );
      },
    );

    Overlay.of(context).insert(overlayEntry!);
  }

  void removeOverlay() {
    overlayEntry?.remove();
    overlayEntry?.dispose();
    overlayEntry = null;
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: createTopBar(context, widget.program.name, actions: [
        IconButton(
          onPressed: () => _onLongPress(context.read<ProgramBloc>().state),
          icon: const Icon(Icons.layers),
        )
      ]),
      body: GestureDetector(
        behavior: HitTestBehavior.opaque,
        onTap: () => _onLongPress(context.read<ProgramBloc>().state),
        onTapCancel: () => _onLongPress(context.read<ProgramBloc>().state),
        child: BlocBuilder<ProgramBloc, ProgramState>(
          builder: (context, state) {
            return Stack(
              children: [
                if (state is ProgramStarted)
                  TerminalView(
                    state.session.terminal,
                    controller: state.session.terminalController,
                    onTapUp: (_, a) => _onLongPress(state),
                    onSecondaryTapDown: (_, a) => _onLongPress(state),
                  ),
                if (state is ProgramStopped)
                  Container(
                    color: Colors.black,
                    child: const Center(
                      child: Text('Press the play button to start the program'),
                    ),
                  ),
              ],
            );
          },
        ),
      ),
    );
  }
}

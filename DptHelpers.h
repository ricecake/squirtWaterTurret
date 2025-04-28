// class SystemState {
// 	motor a
// 	motor b
// 	move target
// 	fire state
// }

class Command {
public:
  int64_t run_after = 0;
  virtual void Execute(SystemState state) = 0;
}

class MoveCommand : Command {

}

struct MoveCmd {
	int H;
	int V;
	int S;
	int D;
	MoveCmd(int h = 0, int v = 0, int s = maxSpeed, int d = 50)
	  : H(h), V(v), S(s), D(d) {}
};

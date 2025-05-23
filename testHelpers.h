/*void initSpeedTest()
{
	for (auto i = 25; i < maxSpeed * 2; i += 25)
	{
		positions.push_back(MoveCmd(h_max, 0, i));
		positions.push_back(MoveCmd(h_min, 0, i));
		positions.push_back(MoveCmd(0, 0, maxSpeed, 100));

		positions.push_back(MoveCmd(0, v_min, i));
		positions.push_back(MoveCmd(0, v_max, i));
		positions.push_back(MoveCmd(0, 0, maxSpeed, 100));
	}
}
void initDemoTest()
{
	positions.push_back(MoveCmd(h_max, 0, maxSpeed / 4, 750));
	positions.push_back(MoveCmd(h_min, 0, maxSpeed / 4, 750));
	positions.push_back(MoveCmd(0, 0, maxSpeed / 4, 750));
	positions.push_back(MoveCmd(0, v_min, maxSpeed / 4, 750));
	positions.push_back(MoveCmd(0, v_max, maxSpeed / 4, 750));
	positions.push_back(MoveCmd(0, 0, maxSpeed / 4, 1000));

	for (auto i = 0; i < 2; i++)
	{
		positions.push_back(MoveCmd(h_max, 0, maxSpeed, 100));
		positions.push_back(MoveCmd(h_min, 0, maxSpeed, 100));
		positions.push_back(MoveCmd(0, 0, maxSpeed, 250));
		positions.push_back(MoveCmd(0, v_min, maxSpeed, 100));
		positions.push_back(MoveCmd(0, v_max, maxSpeed, 100));
		positions.push_back(MoveCmd(0, 0, maxSpeed, 250));
	}
}

void initRangeTest()
{
	positions.push_back(MoveCmd(h_max, v_max, maxSpeed / 2));
	positions.push_back(MoveCmd(h_max, v_min, maxSpeed / 2));

	positions.push_back(MoveCmd(0, 0, maxSpeed, 500));

	positions.push_back(MoveCmd(0, v_min, maxSpeed / 2));
	positions.push_back(MoveCmd(0, v_max, maxSpeed / 2));

	positions.push_back(MoveCmd(0, 0, maxSpeed, 500));

	positions.push_back(MoveCmd(h_min, v_max, maxSpeed / 2));
	positions.push_back(MoveCmd(h_min, v_min, maxSpeed / 2));

	positions.push_back(MoveCmd(0, 0, maxSpeed, 500));

	positions.push_back(MoveCmd(h_max, v_max, maxSpeed / 2));
	positions.push_back(MoveCmd(h_min, v_max, maxSpeed / 2));

	positions.push_back(MoveCmd(0, 0, maxSpeed, 500));

	positions.push_back(MoveCmd(h_min, 0, maxSpeed / 2));
	positions.push_back(MoveCmd(h_max, 0, maxSpeed / 2));

	positions.push_back(MoveCmd(0, 0, maxSpeed, 500));

	positions.push_back(MoveCmd(h_max, v_min, maxSpeed / 2));
	positions.push_back(MoveCmd(h_min, v_min, maxSpeed / 2));

	positions.push_back(MoveCmd(0, 0, maxSpeed, 500));
}

void initOscilateTest()
{
	for (auto i = 0; i < 10; i++)
	{
		positions.push_back(MoveCmd(0, v_min, maxSpeed, 0));
		positions.push_back(MoveCmd(0, v_max, maxSpeed, 0));
	}

	for (auto i = 0; i < 10; i++)
	{
		positions.push_back(MoveCmd(h_min, 0, maxSpeed, 0));
		positions.push_back(MoveCmd(h_max, 0, maxSpeed, 0));
	}
}

void initTestData()
{
	// initSpeedTest();
	// positions.push_back(MoveCmd(0, 0, maxSpeed, 2000));

	initRangeTest();
	positions.push_back(MoveCmd(0, 0, maxSpeed, 2000));

	initOscilateTest();
	positions.push_back(MoveCmd(0, 0, maxSpeed, 2000));

	initDemoTest();
	positions.push_back(MoveCmd(0, 0, maxSpeed, 2000));
}
*/
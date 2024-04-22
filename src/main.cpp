#include <iostream>
#include <unordered_map>
#include <unordered_set>
#include <SFML/Graphics.hpp>
#include <functional>
#include <chrono>
#include "tile2d/world.hpp"

using namespace std::chrono;

inline auto startTime = std::chrono::high_resolution_clock::now();

inline std::chrono::high_resolution_clock::time_point nowTp() {
	return std::chrono::high_resolution_clock::now();
}

template<typename measure = std::chrono::seconds>
typename measure::rep now() {
	using namespace std::chrono;

	auto end = nowTp();
	measure dur = duration_cast<measure>(end - startTime);

	return dur.count();
}

template<typename rep, typename period>
rep now() {
	using namespace std::chrono;
	using durationRP = duration<rep, period>;

	auto end = nowTp();
	durationRP dur = duration_cast<durationRP>(end - startTime);

	return dur.count();
}

inline Float nowSeconds() {
	return now<Float, std::chrono::seconds::period>();
}

template<typename ... func_params>
class Ticker {
public:
	using Function = std::function<func_params...>;

	Ticker()
		: rate(60.0f) {}

	void setRate(Float rate) {
		this->rate = rate;
	}

	// first parameter of the function must be a float_t.
	void setFunction(Function function) {
		this->function = function;
	}

	template<typename ... params>
	void update(params&& ... args) {
		if (lastUpdate == 0.0f) {
			lastUpdate = nowSeconds();
		}

		Float now = nowSeconds();
		updateTime = now - lastUpdate;
		callsTodo += updateTime * rate;
		lastUpdate = now;

		while (callsTodo >= 1.0f) {
			callsTodo--;

			now = nowSeconds();
			deltaTime = now - lastTick;
			lastTick = now;

			function(deltaTime, args...);
		}
	}

	Float getDeltaTime() {
		return deltaTime;
	}

	Float getRate() {
		return rate;
	}

private:
	Function function = nullptr;
	Float rate = 0.0;
	Float lastUpdate = 0.0;
	Float updateTime = 0.0;
	Float callsTodo = 0.0;

	Float lastTick = 0.0f;
	Float deltaTime = 0.0f;
};

sf::Vector2f convert(const vec2& vec) {
	return sf::Vector2f(vec.x, vec.y);
}

struct TileData {
	uint32_t renderId;
};

struct TileRenderData {
	uint32_t textureIndex = 0;
	std::array<sf::Vector2f, 4> textureCoords;
	sf::Color color = sf::Color::White;
};

int main() {
	size_t prev = 0;
	t2d::FreeList<int, int> list;
	for (size_t i = 0; i < 1000; i++) {
		if (rand() % 5 < 2 && i != 0) {
			list.erase(prev);
		}
		
		prev = list.insert(i);
	}

	int testSizes[] = {
		200
	};
	int currentTest = 0;
	int maxTests = 1;
	const Float testLength = 500.0f; // 5 seconds

	sf::RenderWindow window(sf::VideoMode({ 800U, 600U }), "Test Field", sf::State::Windowed);
	sf::View view = window.getDefaultView();
	Float zoom = 1.0f;

	sf::VertexArray va(sf::PrimitiveType::Triangles);
	std::array<sf::Texture, 1> textures;
	if (!textures[0].loadFromFile("./terrain.png")) {
		std::cout << "failed to load!\n";
		return false;
	}
	std::array<TileRenderData, 2> tileRenderData;
	tileRenderData[0].textureCoords = {
		sf::Vector2f(0.0f, 0.0f),
		sf::Vector2f(0.0f, 16.0f),
		sf::Vector2f(16.0f, 16.0f),
		sf::Vector2f(16.0f, 0.0f)
	};
	float tileSize = 16.0f;
	float tileX = 3.0f;
	float tileY = 9.0f;
	tileRenderData[1].textureCoords = {
		sf::Vector2f(tileX * tileSize           , tileY * tileSize),
		sf::Vector2f(tileX * tileSize           , tileY * tileSize + tileSize),
		sf::Vector2f(tileX * tileSize + tileSize, tileY * tileSize + tileSize),
		sf::Vector2f(tileX * tileSize + tileSize, tileY * tileSize )
	};

	std::vector<t2d::debug::CollideInfo> debugCollideInfos;

	for (; currentTest < maxTests && window.isOpen(); currentTest++) {
		t2d::World<TileData> world(12);
		std::vector<t2d::TileMap<TileData>*> tileMaps;
		boost::object_pool<t2d::TileMap<TileData>>  tileMapPool;

		world.setGravity({ 0.0f, 9.81 });

		size_t testBodyCount = 400;
		size_t testBodyWidth = 2;
		size_t testBodyHeight = 2;
		float cursorX = -1000.0f;
		float cursorY = -100.0f;
		for (int i = 0; i < testBodyCount; i++) {
			if (cursorX > 4000.0f) {
				cursorY += t2d::tileHeight * testBodyHeight + 10.0f;
				cursorX = -1000.0f;
				//if (cursorY > window.getSize().y)
				//	break;
			}

			auto tileMap = t2d::createTileMap<TileData>(tileMapPool, world);
			t2d::TileProperties<TileData> props;
			/*props.isMultiTile = true;
			props.isMainTile = true;
			props.mutliTile.width = testBodyWidth;
			props.mutliTile.height = testBodyHeight;*/
			props.userData.renderId = 0;
			tileMap.first->addTile({ 0, 0 }, props);
			tileMap.first->addTile({ 0, 1 }, props);
			tileMap.first->addTile({ 1, 0 }, props);
			tileMap.first->addTile({ 1, 1 }, props);
			tileMap.second->setPos({ cursorX, cursorY });
			//tileMap.second->addLinearVel({ rand() % 10, rand() % 10 });
			tileMaps.emplace_back(tileMap.first);
			world.insertIntoTree(tileMap.second);

			cursorX += t2d::tileWidth * testBodyWidth + 10.0f;
		}

		auto playerTileMap = t2d::createTileMap<TileData>(tileMapPool, world);
		{

			t2d::PhysicsTileProperties physicalProps;
			t2d::TileProperties<TileData> props;
			props.userData.renderId = 1;
			int radius = 5;
			playerTileMap.first->beginBulkInsert();
			//for (int y = -radius; y <= radius; y++)
			//	for (int x = -radius; x <= radius; x++)
			//		if (x * x + y * y <= radius * radius)
			//			playerTileMap.first->addTile({ x, y }, props, physicalProps);
			for (int y = -radius; y <= radius; y++)
				for (int x = -radius; x <= radius; x++)
					playerTileMap.first->addTile({ x, y }, props, physicalProps);
			playerTileMap.first->endBulkInsert();
			playerTileMap.second->setPos({ 400.0f, -200.0f });
			tileMaps.emplace_back(playerTileMap.first);
			world.insertIntoTree(playerTileMap.second);
		}

		auto floor = t2d::createTileMap<TileData>(tileMapPool, world);
		{
			t2d::PhysicsTileProperties physicalProps;
			t2d::TileProperties<TileData> props;
			props.isMultiTile = true;
			props.isMainTile = true;
			props.mutliTile.width = 1;
			props.mutliTile.height = 2;
			props.userData.renderId = 1;
			floor.first->beginBulkInsert();
			for (int x = -500; x <= 500; x++) {
				floor.first->addTile({ x, 0 }, props, physicalProps);
			}
			floor.first->endBulkInsert();
			floor.second->setPos({ 400.0f, 100.0f });
			tileMaps.emplace_back(floor.first);
			world.insertIntoTree(floor.second);
			floor.second->setStatic(true);
		}

		bool toggle = false;
		Float cooldown = 0.040f;

		Ticker<void(Float)> tick;
		tick.setRate(60.0f);
		tick.setFunction([&](Float dt) {
			Float moveSpeed = Float(10.0);
			Float cameraMoveSpeed = (1.0f / zoom);
			if (sf::Keyboard::isKeyPressed(sf::Keyboard::Scan::Left)) {
				view.move({ (float)cameraMoveSpeed , Float(0.0) });
			}
			if (sf::Keyboard::isKeyPressed(sf::Keyboard::Scan::Right)) {
				view.move({ (float)-cameraMoveSpeed , Float(0.0) });
			}
			if (sf::Keyboard::isKeyPressed(sf::Keyboard::Scan::Up)) {
				view.move({ Float(0.0), (float)-cameraMoveSpeed });
			}
			if (sf::Keyboard::isKeyPressed(sf::Keyboard::Scan::Down)) {
				view.move({ Float(0.0), (float)cameraMoveSpeed });
			}
			if (sf::Keyboard::isKeyPressed(sf::Keyboard::Scan::Space) && cooldown <= 0.0f) {
				toggle = !toggle;
				cooldown = 0.040f;
			}

			cooldown  -= dt;

			window.setView(view);

			if (toggle)
				return;
			world.update(dt, 6);
			debugCollideInfos = std::move(t2d::debug::collideInfos);

			if (sf::Mouse::isButtonPressed(sf::Mouse::Button::Left)) {
				sf::Vector2f dir = window.mapPixelToCoords(sf::Mouse::getPosition(window)) - convert(playerTileMap.second->getWorldPos());
				dir = dir.normalized();
				playerTileMap.second->addLinearVel(vec2(dir.x, dir.y) * 10.0f);
			}
		});

		Ticker<void(Float)> statsPrint;
		statsPrint.setRate(1.0f);
		statsPrint.setFunction([&](Float dt) {
			std::cout << "Physics World ETAVG(ms): " << world.getExecutionTimeAvg() << '\n';
			std::cout << "DT: " << tick.getDeltaTime() << '\n';
		});

		steady_clock::time_point start = steady_clock::now();
		while(window.isOpen() && duration_cast<seconds>(steady_clock::now() - start).count() < testLength) {
			sf::Event event;
			while(window.pollEvent(event)) {
				switch(event.type) {
				case sf::Event::Closed:
					window.close();
					break;

				case sf::Event::MouseWheelScrolled:
					if (zoom + event.mouseWheelScroll.delta * 0.1f <= 0.0f)
						break;
					zoom += event.mouseWheelScroll.delta * 0.1f;
					view.zoom(1.0f + event.mouseWheelScroll.delta * 0.1f);
					break;

				default:
					break;
				}
			}
		
			tick.update();
			statsPrint.update();

			// draw;
			window.clear();
			if (!sf::Keyboard::isKeyPressed(sf::Keyboard::Scan::L)) {
				va.clear();
				for (auto& tileMap : tileMaps) {
					t2d::TileBody<TileData>& body = tileMap->body();

					for (auto& tile : *tileMap) {
						if (tileMap->getTileProperties(tile).isMultiTile && !tileMap->getTileProperties(tile).isMainTile)
							continue;

						auto vertices = body.getTileWorldOBBOffsetVertices(tile, { 0.0f, 0.0f });
						TileRenderData& renderData = tileRenderData[tileMap->getTileProperties(tile).userData.renderId];

						va.append({ convert(vertices[0]), renderData.color, renderData.textureCoords[0] });
						va.append({ convert(vertices[1]), renderData.color, renderData.textureCoords[1] });
						va.append({ convert(vertices[2]), renderData.color, renderData.textureCoords[2] });

						va.append({ convert(vertices[2]), renderData.color, renderData.textureCoords[2] });
						va.append({ convert(vertices[3]), renderData.color, renderData.textureCoords[3] });
						va.append({ convert(vertices[0]), renderData.color, renderData.textureCoords[0] });

					}
				}

				window.draw(va, &textures[0]);

				//for (auto& tileMap : tileMaps) {
				//	t2d::TileBody<TileData>& body = tileMap->body();

				//	auto tile = body.getLocalAABBWorldVertices(body.getLocalAABB());
				//	sf::ConvexShape aabb(4);
				//	aabb.setFillColor(sf::Color(0, 255, 0, 64));
				//	aabb.setPoint(0, convert(tile[0]));
				//	aabb.setPoint(1, convert(tile[1]));
				//	aabb.setPoint(2, convert(tile[2]));
				//	aabb.setPoint(3, convert(tile[3]));
				//	window.draw(aabb);
				//}

				for (auto& collideInfo : debugCollideInfos) {
					for (auto& manifold : collideInfo.manifolds) {
						sf::CircleShape circle(1.0f);
						circle.setOrigin({ 1.0f, 1.0f });
						circle.setFillColor(sf::Color::Red);
						for (size_t i = 0; i < manifold.count; i++) {
							circle.setPosition(convert(manifold.points[i]));
							window.draw(circle);

							sf::VertexArray lines(sf::PrimitiveType::LineStrip, 2);
							lines[0].position = convert(manifold.points[i]);
							lines[1].position = convert(manifold.points[i] + manifold.normal * 10.0f);
							window.draw(lines);
						}
					}

					if (sf::Keyboard::isKeyPressed(sf::Keyboard::Scan::Z))
						for (auto& chunk : collideInfo.chunkAABBs) {

							sf::ConvexShape aabb(4);
							aabb.setFillColor(sf::Color(255, 0, 0, 48));
							aabb.setPoint(0, convert(chunk[0]));
							aabb.setPoint(1, convert(chunk[1]));
							aabb.setPoint(2, convert(chunk[2]));
							aabb.setPoint(3, convert(chunk[3]));
							window.draw(aabb);
						}

					if (sf::Keyboard::isKeyPressed(sf::Keyboard::Scan::X))
						for (auto& tile : collideInfo.tileAABBs) {

							sf::ConvexShape aabb(4);
							aabb.setFillColor(sf::Color(0, 255, 0, 48));
							aabb.setPoint(0, convert(tile[0]));
							aabb.setPoint(1, convert(tile[1]));
							aabb.setPoint(2, convert(tile[2]));
							aabb.setPoint(3, convert(tile[3]));
							window.draw(aabb);
						}
				}

				if (sf::Keyboard::isKeyPressed(sf::Keyboard::Scan::C))
					for (auto& pair : world.grid().gridMap()) {
						t2d::AABB<Float> cAABB((vec2)pair.first * t2d::tileWidth * 4.0f, (vec2)(pair.first + 1) * t2d::tileWidth * 4.0f);

						sf::ConvexShape aabb(4);
						if (pair.second.elements.size() <= 4)
							aabb.setFillColor(sf::Color(0, 255, 0, 48));
						else if (pair.second.elements.size() <= 6)
							aabb.setFillColor(sf::Color(128, 200, 0, 48));
						else
							aabb.setFillColor(sf::Color(255, 0, 0, 48));
						aabb.setOutlineColor(sf::Color::White);
						aabb.setOutlineThickness(-1.0f);
						aabb.setPoint(0, convert(cAABB.min()));
						aabb.setPoint(1, convert({ cAABB.max().x, cAABB.min().y }));
						aabb.setPoint(2, convert(cAABB.max()));
						aabb.setPoint(3, convert({ cAABB.min().x, cAABB.max().y }));
						window.draw(aabb);
					}

				sf::CircleShape mouseCircle(2.0f);
				mouseCircle.setOrigin({ 1.0f, 1.0f });
				mouseCircle.setFillColor(sf::Color::Red);
				mouseCircle.setPosition((window.mapPixelToCoords(sf::Mouse::getPosition(window))));
				window.draw(mouseCircle);
			}
			window.display();
		}
	}

	return 0;
}
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

	for (; currentTest < maxTests && window.isOpen(); currentTest++) {
		t2d::World<TileData> world(8);
		std::vector<t2d::TileMap<TileData>*> tileMaps;
		boost::object_pool<t2d::TileMap<TileData>>  tileMapPool;

		size_t testBodyCount = 4000;
		size_t testBodyWidth = 1;
		size_t testBodyHeight = 1;
		float cursorX = 0.0f;
		float cursorY = 0.0f;
		for (int i = 0; i < testBodyCount; i++) {
			if (cursorX > window.getSize().x) {
				cursorY += t2d::tileHeight * testBodyHeight + 10.0f;
				cursorX = 0;
				//if (cursorY > window.getSize().y)
				//	break;
			}

			auto tileMap = t2d::createTileMap<TileData>(tileMapPool, world);
			t2d::TileProperties<TileData> props;
			//props.isMultiTile = true;
			props.isMainTile = true;
			props.mutliTile.width = testBodyWidth;
			props.mutliTile.height = testBodyHeight;
			props.userData.renderId = 0;
			tileMap.first->addTile({ 0, 0 }, props);
			tileMap.second->setPos({ cursorX, cursorY });
			tileMaps.emplace_back(tileMap.first);
			world.insertIntoTree(tileMap.second);

			cursorX += t2d::tileWidth * testBodyWidth + 10.0f;
		}

		auto playerTileMap = t2d::createTileMap<TileData>(tileMapPool, world);
		{
			t2d::TileProperties<TileData> props;
			props.userData.renderId = 1;
			int radius = 50;
			playerTileMap.first->beginBulkInsert();
			for (int y = -radius; y <= radius; y++)
				for (int x = -radius; x <= radius; x++)
					if (x * x + y * y <= radius * radius)
						playerTileMap.first->addTile({ x, y }, props);
			playerTileMap.first->endBulkInsert();
			playerTileMap.second->addLinearVel({ 0.0f, 200.0f });
			playerTileMap.second->setPos({ 400.0f, -600.0f });
			playerTileMap.second->setRot(0.45);
			tileMaps.emplace_back(playerTileMap.first);
			world.insertIntoTree(playerTileMap.second);
		}

		Ticker<void(Float)> tick;
		tick.setRate(60.0f);
		tick.setFunction([&](Float dt) {
			world.update(dt, 6, 0.014);

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
			window.setView(view);

			//if (sf::Mouse::isButtonPressed(sf::Mouse::Button::Left)) {
			//	sf::Vector2f dir = window.mapPixelToCoords(sf::Mouse::getPosition(window)) - convert(playerTileMap.second->getWorldPos());
			//	dir = dir.normalized();
			//	playerTileMap.second->addLinearVel(vec2(dir.x, dir.y) * 10.0f);
			//}
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

			va.clear();
			for (auto& tileMap : tileMaps) {
				t2d::TileBody<TileData>& body = tileMap->body();

				for (auto& tile : *tileMap) {
					auto vertices = body.getTileWorldOBB(tile).getVertices();
					TileRenderData& renderData = tileRenderData[tileMap->getTileProperties(tile).userData.renderId];

					va.append({convert(vertices[0]), renderData.color, renderData.textureCoords[0]});
					va.append({convert(vertices[1]), renderData.color, renderData.textureCoords[1]});
					va.append({convert(vertices[2]), renderData.color, renderData.textureCoords[2]});

					va.append({convert(vertices[2]), renderData.color, renderData.textureCoords[2]});
					va.append({convert(vertices[3]), renderData.color, renderData.textureCoords[3]});
					va.append({convert(vertices[0]), renderData.color, renderData.textureCoords[0]});

				}
			}

			window.draw(va, &textures[0]);
		
			sf::CircleShape mouseCircle(2.0f);
			mouseCircle.setOrigin({ 1.0f, 1.0f });
			mouseCircle.setFillColor(sf::Color::Red);
			mouseCircle.setPosition((window.mapPixelToCoords(sf::Mouse::getPosition(window))));
			window.draw(mouseCircle);

			window.display();
		}
	}

	return 0;
}
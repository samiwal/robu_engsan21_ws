
class FuelCalculator:
    def __init__(self, drivenKm, consumptionLt):
        self._drivenKm = drivenKm
        self._consumptionLt = consumptionLt

        if drivenKm <= 0.0:
            raise ValueError("Gefahrene Kilometer müssen größer null sein!")
        if consumptionLt < 0.0:
            raise ValueError("Verbrauch in Liter muss größer minus eins sein")
        self._calc()

    def _calc(self):
        self._avg_consumptionLt = 100 * self._consumptionLt / self._drivenKm

    def get_average_consumption(self):
        return self._avg_consumptionLt
    def __str__(self):
        return f"{'Durchschnittsverbrauch:':25s}{self._avg_consumptionLt:4.2f} l/100 km"
#fuelcalculator = FuelCalculator(1000, 60)
#print("Der durchschnittsverbrauch beträgt: ", fuelcalculator.get_average_consumption())
#print(fuelcalculator)
class FuelUI:
    def __init__(self):
        try:
            self.input()
            self.calc()
            self.output()
        except Exception as e:
            print(e)
    def input(self):
        print("Fuel consumption calculator")
        print("===========================")

        self._drivenKm = float(input(f"{'Kilometer Driven:':25s}"))
        self._consumtionLt = float(input(f"{'Consumption in liter:':25s}"))
    def calc(self):
        self._fuelcalculator = FuelCalculator(self._drivenKm,self._consumtionLt)
    def output(self):
        print(self._fuelcalculator)
if __name__  == "__main__":
    FuelUI()
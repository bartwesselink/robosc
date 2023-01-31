import { ComponentFixture, TestBed } from '@angular/core/testing';

import { OverviewScopeComponent } from './overview-scope.component';

describe('OverviewScopeComponent', () => {
  let component: OverviewScopeComponent;
  let fixture: ComponentFixture<OverviewScopeComponent>;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      declarations: [ OverviewScopeComponent ]
    })
    .compileComponents();

    fixture = TestBed.createComponent(OverviewScopeComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
